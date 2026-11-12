"""Дифпары: длина каждого проводника между выводами на концах пары и перекос.

Длина считается по графу трасс цепи (Дейкстра): узлы — концы отрезков и точки Т-примыканий, переходы
соединяют слои. Концы пары — две наиболее удалённые друг от друга микросхемы/разъёма, у которых есть выводы
обоих проводников (например, USB-C и MCU). Перекос в пикосекундах — по εeff слоя пары.
"""
import heapq, math, collections
from shapely.geometry import LineString, Point


def _graph(tracks, vias, net):
    segs = [t for t in tracks if t[0] == net]
    pts = collections.defaultdict(set)
    for t in segs:
        pts[id(t)] |= {(t[2], t[3]), (t[4], t[5])}
    for t in segs:                                      # Т-примыкания: конец одного отрезка на другом
        line = LineString([(t[2], t[3]), (t[4], t[5])])
        for u in segs:
            if u is t or u[1] != t[1]: continue
            for e in ((u[2], u[3]), (u[4], u[5])):
                if line.distance(Point(e)) < 1e-4: pts[id(t)].add(e)
    adj = collections.defaultdict(list)
    def node(x, y, l): return (round(x, 3), round(y, 3), l)
    for t in segs:
        line = LineString([(t[2], t[3]), (t[4], t[5])])
        ps = sorted(pts[id(t)], key=lambda p: line.project(Point(p)))
        for a, b in zip(ps, ps[1:]):
            d = math.hypot(b[0] - a[0], b[1] - a[1])
            na, nb = node(*a, t[1]), node(*b, t[1])
            adj[na].append((nb, d)); adj[nb].append((na, d))
    for v in vias:
        if v[0] != net: continue
        ns = [k for k in list(adj) if abs(k[0] - v[1]) < 0.31 and abs(k[1] - v[2]) < 0.31]
        for a in ns:
            for b in ns:
                if a != b: adj[a].append((b, 0.0))
    return adj


def path_length(tracks, vias, net, pa, pb):
    adj = _graph(tracks, vias, net)
    def near(p):
        return [k for k in adj if math.hypot(k[0] - p.x, k[1] - p.y) < max(p.w, p.h) / 2 + 0.05]
    src = near(pa); dst = set(near(pb))
    if not src or not dst: return None
    dist = {}; pq = [(math.hypot(k[0] - pa.x, k[1] - pa.y), k) for k in src]
    heapq.heapify(pq)
    while pq:
        d, k = heapq.heappop(pq)
        if k in dist: continue
        dist[k] = d
        if k in dst: return d + math.hypot(k[0] - pb.x, k[1] - pb.y)
        for nb, w in adj[k]:
            if nb not in dist: heapq.heappush(pq, (d + w, nb))
    return None


def _ends(R, P, N):
    refs = collections.defaultdict(lambda: {P: [], N: []})
    for p in R.pads:
        if p.net in (P, N): refs[p.ref][p.net].append(p)
    ends = [r for r, d in refs.items() if d[P] and d[N]]
    if len(ends) < 2: return None, None, refs
    def c(r): ps = refs[r][P] + refs[r][N]; return (sum(p.x for p in ps) / len(ps), sum(p.y for p in ps) / len(ps))
    a, b = max(((a, b) for a in ends for b in ends if a < b), key=lambda ab: math.dist(c(ab[0]), c(ab[1])))
    return a, b, refs


def _length(R, net, a, b, refs):
    tr = [tuple(t[:7]) for t in R.tracks if t is not None]; vi = [tuple(v[:3]) for v in R.vias if v is not None]
    ls = [path_length(tr, vi, net, pa, pb) for pa in refs[a][net] for pb in refs[b][net]]
    ls = [x for x in ls if x is not None]
    return min(ls) if ls else None


def compensate(R, log=print):
    """Выровнять длины проводников пары: на прямом участке короткого проводника — «змейка» из П-образных
    изгибов в сторону от напарника; высота подбирается под недостающую длину. Каждый отрезок проверяется на зазоры."""
    for info in R.rules.pair_info:
        P, N = info['p'], info['n']
        a, b, refs = _ends(R, P, N)
        if a is None: continue
        lp, ln = _length(R, P, a, b, refs), _length(R, N, a, b, refs)
        if lp is None or ln is None: continue
        need = abs(lp - ln); tol = R.cfg['nets']['skew_max']
        if need <= tol: continue
        short, other = (P, N) if lp < ln else (N, P)
        w = info['w']; pitch = max(3 * w, w + R.rules.v['clearance'] + 0.1)     # шаг ног змейки
        part = [t for t in R.tracks if t is not None and t[0] == other]
        from shapely.geometry import LineString, Point
        from shapely.ops import unary_union
        pg = unary_union([LineString([(t[2], t[3]), (t[4], t[5])]) for t in part]) if part else None
        done = False
        def octi(t):
            dx, dy = abs(t[4] - t[2]), abs(t[5] - t[3])
            return dx < 1e-9 or dy < 1e-9 or abs(dx - dy) < 1e-6
        cands = sorted([k for k, t in enumerate(R.tracks) if t is not None and t[0] == short and t[7] == 'net' and octi(t)],
                       key=lambda k: -math.hypot(R.tracks[k][4] - R.tracks[k][2], R.tracks[k][5] - R.tracks[k][3]))
        for k in cands:
            t = R.tracks[k]; L = math.hypot(t[4] - t[2], t[5] - t[3])
            ux, uy = (t[4] - t[2]) / L, (t[5] - t[3]) / L
            for amp in (1.0, 0.7, 0.5, 0.35):
                nb = math.ceil(need / (2 * amp))
                span = nb * 2 * pitch
                if span + 2 * pitch > L: continue
                starts = [pitch + 0.5 * i for i in range(int((L - span - 2 * pitch) / 0.5) + 1)]
                starts.sort(key=lambda s: abs(s - (L - span) / 2))          # от середины к краям
                for side, s0 in [(sd, s) for sd in (1, -1) for s in starts]:
                    nx, ny = -uy * side, ux * side
                    mid = Point(t[2] + ux * (s0 + span / 2) + nx * amp, t[3] + uy * (s0 + span / 2) + ny * amp)
                    if pg is not None and pg.distance(mid) < amp:      # в сторону напарника — нельзя
                        continue
                    pts = [(t[2], t[3])]
                    rem = need
                    for i in range(nb):
                        h = min(amp, rem / 2); rem -= 2 * h
                        x0 = t[2] + ux * (s0 + i * 2 * pitch); y0 = t[3] + uy * (s0 + i * 2 * pitch)
                        x1 = x0 + ux * pitch; y1 = y0 + uy * pitch
                        pts += [(x0, y0), (x0 + nx * h, y0 + ny * h), (x1 + nx * h, y1 + ny * h), (x1, y1)]
                    pts.append((t[4], t[5]))
                    pts = [p for i, p in enumerate(pts) if i == 0 or math.dist(p, pts[i - 1]) > 1e-6]
                    if all(R._clear_ok(short, t[1], pa, pb, w) for pa, pb in zip(pts, pts[1:])):
                        R.tracks[k] = None
                        for pa, pb in zip(pts, pts[1:]):
                            R.add_track(short, t[1], pa[0], pa[1], pb[0], pb[1], t[6], 'net')
                        done = True; break
                if done: break
            if done: break
        lp2, ln2 = _length(R, P, a, b, refs), _length(R, N, a, b, refs)
        log('  пара %s/%s: перекос %.2f → %s мм%s' % (P, N, need, None if lp2 is None or ln2 is None else round(abs(lp2 - ln2), 3),
                                                     '' if done else ' (нет места под змейку)'))


def report(R, tracks, vias):
    out = []
    for info in R.rules.pair_info:
        P, N = info['p'], info['n']
        refs = collections.defaultdict(lambda: {P: [], N: []})
        for p in R.pads:
            if p.net in (P, N): refs[p.ref][p.net].append(p)
        ends = [r for r, d in refs.items() if d[P] and d[N]]
        if len(ends) < 2:
            out.append(dict(pair=(P, N), note='у пары меньше двух общих концов')); continue
        def c(r): ps = refs[r][P] + refs[r][N]; return (sum(p.x for p in ps) / len(ps), sum(p.y for p in ps) / len(ps))
        a, b = max(((a, b) for a in ends for b in ends if a < b), key=lambda ab: math.dist(c(ab[0]), c(ab[1])))
        # у разъёма (USB-C) у проводника может быть несколько площадок: берётся кратчайший путь между концами
        def best(net):
            ls = [path_length(tracks, vias, net, pa, pb) for pa in refs[a][net] for pb in refs[b][net]]
            ls = [x for x in ls if x is not None]
            return min(ls) if ls else None
        lp, ln = best(P), best(N)
        d = dict(pair=(P, N), ends=(a, b), w=info['w'], s=info['s'], zdiff=round(info['z'], 1), target=info['target'],
                 len_p=None if lp is None else round(lp, 2), len_n=None if ln is None else round(ln, 2))
        if lp is not None and ln is not None:
            d['skew_mm'] = round(abs(lp - ln), 3); d['skew_ps'] = round(abs(lp - ln) * info['ps_mm'], 1)
            d['ok'] = d['skew_mm'] <= R.cfg['nets']['skew_max']
        out.append(d)
    return out
