// Octilinear A* with bend penalties, per-layer direction costs, per-cell and per-via cost maps.
// State = (layer, j, i, dir) ; dir 0..7 arrival direction, 8 = start / after via.
// Portable C99 + thread-local storage: builds with GCC, Clang (zig cc) and MSVC; safe to call from several threads.
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>

#if defined(_MSC_VER)
#define TLS __declspec(thread)
#define EXPORT __declspec(dllexport)
#elif defined(_WIN32)
#define TLS __thread
#define EXPORT __declspec(dllexport)
#else
#define TLS __thread
#define EXPORT
#endif

typedef struct { float f; float h; int s; } HN;

/* per-thread work buffers, reused between calls; a state is "visited" only if stamp[s] == gen,
   so nothing has to be cleared between searches */
static TLS HN *heap; static TLS int hn, hcap;
static TLS float *g_g; static TLS int *g_prev; static TLS uint32_t *g_stamp; static TLS long g_cap;
static TLS uint32_t g_gen;
static TLS unsigned char *b_seen; static TLS int *b_q; static TLS long b_cap;

static int ensure(long N) {
    if (N <= g_cap) return 1;
    free(g_g); free(g_prev); free(g_stamp);
    g_g = (float *)malloc(sizeof(float) * N); g_prev = (int *)malloc(sizeof(int) * N);
    g_stamp = (uint32_t *)calloc(N, sizeof(uint32_t));
    if (!g_g || !g_prev || !g_stamp) { free(g_g); free(g_prev); free(g_stamp); g_g = 0; g_prev = 0; g_stamp = 0; g_cap = 0; return 0; }
    g_cap = N; g_gen = 0;
    return 1;
}

/* ties on f are broken towards the node closer to the goal (smaller h):
   the search runs straight at the target instead of flooding equal-cost zigzags */
static int hless(const HN *a, const HN *b) { return a->f < b->f || (a->f == b->f && a->h < b->h); }

static void hpush(float f, float h, int s) {
    if (hn >= hcap) { hcap = hcap ? hcap * 2 : 1 << 16; heap = (HN *)realloc(heap, sizeof(HN) * hcap); }
    HN x; x.f = f; x.h = h; x.s = s;
    int k = hn++;
    while (k > 0) { int p = (k - 1) >> 1; if (!hless(&x, &heap[p])) break; heap[k] = heap[p]; k = p; }
    heap[k] = x;
}
static HN hpop(void) {
    HN top = heap[0]; HN last = heap[--hn]; int k = 0;
    for (;;) {
        int c = 2 * k + 1; if (c >= hn) break;
        if (c + 1 < hn && hless(&heap[c + 1], &heap[c])) c++;
        if (!hless(&heap[c], &last)) break;
        heap[k] = heap[c]; k = c;
    }
    if (hn) heap[k] = last;
    return top;
}

static const int DI[8] = {1, 1, 0, -1, -1, -1, 0, 1};
static const int DJ[8] = {0, 1, 1, 1, 0, -1, -1, -1};
static const float SQ2 = 1.41421356f;

typedef struct { int tx0, tx1, ty0, ty1; float hmin; } HC;
static float heur(const HC *c, int i, int j) {
    int dx = i < c->tx0 ? c->tx0 - i : (i > c->tx1 ? i - c->tx1 : 0);
    int dy = j < c->ty0 ? c->ty0 - j : (j > c->ty1 ? j - c->ty1 : 0);
    float mx = (float)(dx > dy ? dx : dy), mn = (float)(dx > dy ? dy : dx);
    return c->hmin * (mx + (SQ2 - 1.0f) * mn);
}

/* Cheap reachability test: BFS over cells (no direction states) with the same move rules.
   A failing A* explores the whole window (millions of states); this answers "no path" in milliseconds. */
static int reachable(int W, int H, int L, const unsigned char *allow, const unsigned char *viaok,
                     const unsigned char *tgt, int nsrc, const int *src)
{
    long N = (long)L * H * W;
    if (N > b_cap) {
        free(b_seen); free(b_q);
        b_seen = (unsigned char *)malloc(N); b_q = (int *)malloc(sizeof(int) * N);
        if (!b_seen || !b_q) { free(b_seen); free(b_q); b_seen = 0; b_q = 0; b_cap = 0; return 1; }
        b_cap = N;
    }
    memset(b_seen, 0, N);
    long qh = 0, qt = 0; int found = 0;
    for (int k = 0; k < nsrc; k++) {
        int c = (src[3 * k] * H + src[3 * k + 1]) * W + src[3 * k + 2];
        if (!b_seen[c]) { b_seen[c] = 1; b_q[qt++] = c; }
    }
    while (qh < qt) {
        int c = b_q[qh++];
        if (tgt[c]) { found = 1; break; }
        int i = c % W, j = (c / W) % H, l = c / (W * H);
        for (int nd = 0; nd < 8; nd++) {
            int ii = i + DI[nd], jj = j + DJ[nd];
            if (ii < 0 || jj < 0 || ii >= W || jj >= H) continue;
            int nc = (l * H + jj) * W + ii;
            if (b_seen[nc] || !allow[nc]) continue;
            if (DI[nd] && DJ[nd] && !allow[(l * H + j) * W + ii] && !allow[(l * H + jj) * W + i]) continue;
            b_seen[nc] = 1; b_q[qt++] = nc;
        }
        if (viaok[j * W + i])
            for (int l2 = 0; l2 < L; l2++) {
                int nc = (l2 * H + j) * W + i;
                if (l2 != l && !b_seen[nc] && allow[nc]) { b_seen[nc] = 1; b_q[qt++] = nc; }
            }
    }
    return found;
}

/* allow  : L*H*W, 1 = cell centre may carry this net's track on that layer
   viaok  : H*W,   1 = a via may stand here
   tgt    : L*H*W, 1 = target cell
   src    : nsrc triplets (l, j, i)
   dircost: L*3 multipliers (horizontal, vertical, diagonal)
   cost   : L*H*W extra cost of entering a cell (NULL = none): bus lanes, congestion (PathFinder)
   viacost: H*W extra cost of a via at a cell (NULL = none): congestion on all layers it crosses
   hw     : heuristic weight; 1 = optimal A*, >1 = weighted A* (faster, path cost <= hw * optimum)
   returns number of cells written to out (triplets l,j,i from source to target), -1 none, -2 no memory */
EXPORT int astar(int W, int H, int L, const unsigned char *allow, const unsigned char *viaok,
          const unsigned char *tgt, int nsrc, const int *src,
          const float *dircost, float b45, float b90, float viac,
          int tx0, int tx1, int ty0, int ty1, int maxexp, int *out, int cap, const float *cost, float hw,
          const float *viacost)
{
    if (!reachable(W, H, L, allow, viaok, tgt, nsrc, src)) return -1;
    long N = (long)L * H * W * 9;
    if (!ensure(N)) return -2;
    if (++g_gen == 0) { memset(g_stamp, 0, sizeof(uint32_t) * g_cap); g_gen = 1; }
    const uint32_t gen = g_gen;
    float *g = g_g; int *prev = g_prev; uint32_t *st = g_stamp;
    hn = 0;
    HC hc; hc.tx0 = tx0; hc.tx1 = tx1; hc.ty0 = ty0; hc.ty1 = ty1; hc.hmin = 1e30f;
    for (int k = 0; k < L * 3; k++) if (dircost[k] < hc.hmin) hc.hmin = dircost[k];
    if (hc.hmin > 1.0f) hc.hmin = 1.0f;
    if (hw < 1.0f) hw = 1.0f;
    hc.hmin *= hw;
#define G(s) (st[s] == gen ? g[s] : 1e30f)
    for (int k = 0; k < nsrc; k++) {
        int l = src[3 * k], j = src[3 * k + 1], i = src[3 * k + 2];
        long s = (((long)l * H + j) * W + i) * 9 + 8;
        float h = heur(&hc, i, j);
        g[s] = 0; prev[s] = -1; st[s] = gen; hpush(h, h, (int)s);
    }
    int goal = -1, nexp = 0;
    while (hn) {
        HN t = hpop(); int s = t.s;
        int d = s % 9; int cell = s / 9; int i = cell % W; int j = (cell / W) % H; int l = cell / (W * H);
        float gc = g[s];
        if (t.f - t.h > gc + 1e-4f) continue;          /* stale heap entry */
        if (tgt[cell]) { goal = s; break; }
        if (++nexp > maxexp) break;
        const float *dc = dircost + 3 * l;
        for (int nd = 0; nd < 8; nd++) {
            float bend = 0;
            if (d < 8) {
                int a = abs(nd - d); if (a > 4) a = 8 - a;
                if (a >= 3) continue;                   /* no turns sharper than 90 deg */
                bend = a == 1 ? b45 : (a == 2 ? b90 : 0);
            }
            int ii = i + DI[nd], jj = j + DJ[nd];
            if (ii < 0 || jj < 0 || ii >= W || jj >= H) continue;
            int nc = (l * H + jj) * W + ii;
            if (!allow[nc]) continue;
            float step;
            if (DI[nd] && DJ[nd]) {
                /* diagonal: at least one side cell must be free (the grid keeps CLR+MARG at cell centres) */
                if (!allow[(l * H + j) * W + ii] && !allow[(l * H + jj) * W + i]) continue;
                step = SQ2 * dc[2];
            } else step = DI[nd] ? dc[0] : dc[1];
            if (cost) step += (DI[nd] && DJ[nd] ? SQ2 : 1.0f) * cost[nc];
            long ns = (long)nc * 9 + nd; float ng = gc + step + bend;
            if (ng < G(ns)) { float h = heur(&hc, ii, jj); g[ns] = ng; prev[ns] = s; st[ns] = gen; hpush(ng + h, h, (int)ns); }
        }
        if (viaok[j * W + i]) {
            float vc = viac + (viacost ? viacost[j * W + i] : 0.0f);
            for (int l2 = 0; l2 < L; l2++) {
                if (l2 == l) continue;
                int nc = (l2 * H + j) * W + i;
                if (!allow[nc]) continue;
                long ns = (long)nc * 9 + 8; float ng = gc + vc;
                if (ng < G(ns)) { float h = heur(&hc, i, j); g[ns] = ng; prev[ns] = s; st[ns] = gen; hpush(ng + h, h, (int)ns); }
            }
        }
    }
#undef G
    int n = -1;
    if (goal >= 0) {
        n = 0; int s = goal;
        while (s >= 0 && n < cap) {
            int cell = s / 9; out[3 * n] = cell / (W * H); out[3 * n + 1] = (cell / W) % H; out[3 * n + 2] = cell % W; n++;
            s = prev[s];
        }
        for (int a = 0, b = n - 1; a < b; a++, b--)
            for (int c = 0; c < 3; c++) { int tmp = out[3 * a + c]; out[3 * a + c] = out[3 * b + c]; out[3 * b + c] = tmp; }
    }
    return n;
}
