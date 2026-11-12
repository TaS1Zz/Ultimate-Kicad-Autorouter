"""Октилинейный маршрутизатор v3 (Router2).

  * правила из платы и конфига (rules.Rules): контур, стек, зазоры, переходы, токи, быстрые цепи, дифпары;
  * ширина по слоям (ток, падение напряжения), сужение только у узких выводов (neck-down);
  * спрямление пути после A*, точный вход в центр площадки;
  * топология цепи — минимальное остовное дерево от вывода ведущей микросхемы (матрица идёт цепочкой);
  * шины: цепи группы притягиваются к полосе «ширина + зазор» от соседей (параллельно, на одном слое);
  * PathFinder (negotiated congestion): сигнальные цепи сначала «мягкие» — могут перекрываться, но платят за
    чужую территорию (PF·занятость + история). PF растёт с каждым проходом, история копится в местах конфликтов,
    и цепи расходятся сами — без rip-up по очереди и зависимости от порядка;
  * дифпары: второй проводник идёт полосой на шаге w+s от первого с зазором s внутри пары;
  * fanout: глубина перехода выбирается по загрузке соседних каналов, а не по чётности номера вывода;
  * контекст поиска — свой у каждого потока: независимые цепи можно разводить параллельно.
"""
import math, ctypes, uuid, os, re, sys, threading
import numpy as np
from board import load, CU
import route as G
from route import Maps, circ_dist, seg_dist, pad_shape, RES, MARG, CAP
from rules import Rules, bus_groups

_LIBNAME = {"win32": "astar.dll", "darwin": "libastar.dylib"}.get(sys.platform, "libastar.so")
_lib = ctypes.CDLL(os.path.join(os.path.dirname(os.path.abspath(__file__)), _LIBNAME))   # собрать: build.sh / build.bat
_lib.astar.restype = ctypes.c_int
P_u8 = ctypes.POINTER(ctypes.c_ubyte); P_i = ctypes.POINTER(ctypes.c_int); P_f = ctypes.POINTER(ctypes.c_float)
_lib.astar.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.c_int, P_u8, P_u8, P_u8, ctypes.c_int, P_i,
                       P_f, ctypes.c_float, ctypes.c_float, ctypes.c_float,
                       ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_int, P_i, ctypes.c_int,
                       ctypes.c_void_p, ctypes.c_float, ctypes.c_void_p]

RL = [0, 2, 3]                      # F.Cu, In2.Cu, B.Cu; In1 — сплошная GND
# (горизонталь, вертикаль, диагональ): In2 — горизонтальные шины, B — вертикальные спуски,
# F — короткие подходы к площадкам. Диагональ дороже ортогонали: 45° только там, где сокращает путь.
DIRC = {0: (1.0, 1.5, 1.25), 2: (1.0, 2.0, 1.5), 3: (2.0, 1.0, 1.5)}
B45, B90 = 12.0, 30.0               # штраф за излом, клеток по 0.1 мм; 90° дороже двух 45°
VIAC = 60.0                         # цена перехода, клеток (6 мм пути): переходов меньше — меньше индуктивность и отверстий
SMOOTH_BEND = 40.0                  # цена излома 45° при спрямлении, клеток (90° — ×2.2)
LANE_COST = 0.3                     # надбавка за клетку вне полосы шины
LANE_R = 1.6                        # полоса шины «чувствуется» в этом радиусе, мм
LANE_HW = 1.5                       # вес эвристики при поиске с полосой/перегрузкой (взвешенный A*: в разы быстрее)
PAIR_COST = 2.0                     # надбавка за клетку вне полосы дифпары (пара держится вместе сильнее шины)
NECK_R = 0.8                        # зона сужения вокруг узкой площадки, мм
THERMAL_KEEP = 0.3                  # доп. отступ чужих трасс от штыревых GND-площадок на F/B (мостики терморазгрузки), мм
PF0, PF_MUL = 0.6, 1.6              # PathFinder: начальная цена чужой территории и её рост за проход
HINC = 0.6                          # PathFinder: прибавка истории в клетке конфликта за проход
ESC_COST = 4.0                      # цена клетки чужого коридора выхода из вывода микросхемы
ESC_LEN = 1.2                       # длина коридора выхода за край площадки, мм

# параметры можно переопределить без правки кода: ROUTER_VIAC=80 python route_board.py ...
for _k in ('B45', 'B90', 'VIAC', 'SMOOTH_BEND', 'LANE_COST', 'LANE_R', 'LANE_HW', 'PAIR_COST', 'NECK_R',
           'THERMAL_KEEP', 'PF0', 'PF_MUL', 'HINC', 'ESC_COST', 'ESC_LEN'):
    if os.environ.get('ROUTER_' + _k): globals()[_k] = float(os.environ['ROUTER_' + _k])

D8 = [(0, 1), (1, 1), (1, 0), (1, -1), (0, -1), (-1, -1), (-1, 0), (-1, 1)]   # (dj, di)


def _sgn(v): return int(v > 0) - int(v < 0)


def octile(ax, ay, bx, by):
    dx, dy = abs(ax - bx), abs(ay - by)
    return max(dx, dy) + (math.sqrt(2) - 1) * min(dx, dy)


class Router2:
    def __init__(self, path, cfg=None):
        self.rules = Rules(path, cfg)
        self.cfg = self.rules.cfg
        self.root, self.pads = load(path)
        G.configure(self.rules, self.pads)
        self.netid, self.netname = {}, {}
        uniq = 100000
        for p in self.pads:
            if p.net:
                if p.net not in self.netid:
                    k = len(self.netid) + 1; self.netid[p.net] = k; self.netname[k] = p.net
                p.nid = self.netid[p.net]
            else:
                uniq += 1; p.nid = uniq
        self.groups = bus_groups(self.netid.keys(), exclude=set(self.rules.pair))
        self.group_of = {n: g for g, ms in self.groups.items() for n in ms}
        self.tracks = []    # [name, layer, ax, ay, bx, by, w, tag]
        self.vias = []      # [name, x, y, tag]
        self.fail = []
        self.warn = []      # цепи, проведённые уже допустимой по нагреву, но не целевой ширины
        self.anchors = {}   # net -> set of (l, j, i) cells on track/via centrelines
        self.fan = {}       # pad key -> list of anchor cells of its fanout
        self.fanrec = {}
        # PathFinder
        self.soft = set()   # «мягкие» цепи: их медь — не препятствие, а территория с ценой
        self.occ = np.zeros((4, G.H, G.W), np.int16)
        self.hist = np.zeros((4, G.H, G.W), np.float32)
        self.terr = {}      # net -> {layer: flat cell indices}
        self.pf = 0.0
        self._tl = threading.local()
        self.lock = threading.RLock()
        self._escape_corridors()
        self.rebuild()

    def _escape_corridors(self):
        """Коридоры выхода: полоса шириной с площадку и длиной ESC_LEN наружу от каждого вывода микросхемы
        с мелким шагом. Чужие цепи платят за проход по ней ESC_COST — поэтому дифпара или питание, разведённые
        первыми, огибают выводы соседей, а не замуровывают их (иначе выводу некуда выйти)."""
        self.esc = np.zeros((4, G.H, G.W), np.int32)
        byref = {}
        for p in self.pads: byref.setdefault(p.ref, []).append(p)
        for ref, ps in byref.items():
            fine = [p for p in ps if p.kind == 'smd' and p.net and min(p.w, p.h) <= 0.35]
            if len(fine) < 4: continue
            cx = sum(p.x for p in ps) / len(ps); cy = sum(p.y for p in ps) / len(ps)
            for p in fine:
                horiz = p.w > p.h if round(p.ang) % 180 == 0 else p.h > p.w   # длинная сторона площадки по x
                L, Wd = max(p.w, p.h), min(p.w, p.h)
                if horiz:
                    s = 1 if p.x > cx else -1
                    x0, x1 = sorted((p.x + s * L / 2, p.x + s * (L / 2 + ESC_LEN))); y0, y1 = p.y - Wd / 2, p.y + Wd / 2
                else:
                    s = 1 if p.y > cy else -1
                    y0, y1 = sorted((p.y + s * L / 2, p.y + s * (L / 2 + ESC_LEN))); x0, x1 = p.x - Wd / 2, p.x + Wd / 2
                i0, i1, j0, j1 = self.maps_window(x0, y0, x1, y1)
                for l in p.layers: self.esc[CU.index(l), j0:j1, i0:i1] = p.nid

    @staticmethod
    def maps_window(x0, y0, x1, y1):
        i0 = max(0, int(math.floor((x0 - G.X0) / RES))); i1 = min(G.W, int(math.ceil((x1 - G.X0) / RES)) + 1)
        j0 = max(0, int(math.floor((y0 - G.Y0) / RES))); j1 = min(G.H, int(math.ceil((y1 - G.Y0) / RES)) + 1)
        return i0, i1, j0, j1

    @property
    def _ctx(self): return self._tl.ctx

    @_ctx.setter
    def _ctx(self, v): self._tl.ctx = v

    # ------------------------------------------------------------------ maps
    def build_pads(self):
        for p in self.pads:
            if p.kind == 'np_thru_hole':
                r = (p.drill if p.drill else p.w) / 2
                r = 2.0 if p.ref.startswith('MH') else r + 0.05
                f, bb = circ_dist(p.x, p.y, r)
                for l in range(4): self.maps.add(l, p.nid, bb, f)
                continue
            f, bb = pad_shape(p)
            for l in p.layers: self.maps.add(CU.index(l), p.nid, bb, f)
            # штыревые GND-площадки (разъёмы) на внешних слоях: место под мостики терморазгрузки заливки —
            # чужие трассы держатся дальше на THERMAL_KEEP (иначе KiCad: starved_thermal).
            # У SMD-выводов микросхем с мелким шагом отступа нет: там он перекрывает fanout соседей.
            keep = getattr(self, 'keep_extra', {}).get(self.pkey(p), THERMAL_KEEP if p.kind == 'thru_hole' else 0.0)
            if p.net == 'GND' and keep > 0:
                g = (lambda f, k: (lambda X, Y: f(X, Y) - k))(f, keep)
                bb2 = (bb[0] - keep, bb[1] - keep, bb[2] + keep, bb[3] + keep)
                for l in p.layers:
                    if CU.index(l) in (0, 3): self.maps.add(CU.index(l), p.nid, bb2, g)

    def rebuild(self):
        self.maps = Maps(); self.build_pads()
        for t in self.tracks:
            if t is None or t[0] in self.soft: continue
            f, bb = seg_dist(t[2], t[3], t[4], t[5], t[6]); self.maps.add(t[1], self.netid[t[0]], bb, f)
        for v in self.vias:
            if v is None or v[0] in self.soft: continue
            f, bb = circ_dist(v[1], v[2], G.VIA_D / 2)
            for l in range(4): self.maps.add(l, self.netid[v[0]], bb, f)
        self.occ[:] = 0; self.terr = {}
        for n in self.soft: self.add_territory(n)

    def allowed1(self, nid, l, h, win, pair=None):
        i0, i1, j0, j1 = win; sl = (slice(j0, j1), slice(i0, i1))
        edge = self.maps.edge[sl] >= G.EDGE + h + MARG
        if pair is None:
            return (self.maps.dn(l, nid, sl) >= G.CLR + h + MARG) & edge
        # дифпара: до напарника — зазор пары s, до остальных — обычный (оценка с запасом:
        # если ближайшая медь — напарник или своя, берётся следующая по близости цепь)
        pnid, s, dpart = pair
        n1 = self.maps.n1[l][sl]; d1 = self.maps.d1[l][sl]; d2 = self.maps.d2[l][sl]
        d_other = np.where((n1 == pnid) | (n1 == nid), d2, d1)
        return (d_other >= G.CLR + h + MARG) & (dpart[l] >= s + h + MARG) & edge

    # ------------------------------------------------------------------ PathFinder territory
    def _terr_cells(self, name):
        r_sig = self.rules.v['track'] / 2
        cells = {l: [] for l in range(4)}
        for t in self.tracks:
            if t is None or t[0] != name: continue
            r = t[6] / 2 + G.CLR + r_sig + MARG + 0.04      # +0.04: диагональный шаг срезает угол клетки
            i0, i1, j0, j1 = self.maps.window(min(t[2], t[4]) - r, min(t[3], t[5]) - r, max(t[2], t[4]) + r, max(t[3], t[5]) + r)
            if i0 >= i1 or j0 >= j1: continue
            X, Y = np.meshgrid(G.xs[i0:i1], G.ys[j0:j1])
            f, _ = seg_dist(t[2], t[3], t[4], t[5], 0.0)
            jj, ii = np.nonzero(f(X, Y) <= r)
            cells[t[1]].append((jj + j0) * G.W + (ii + i0))
        for v in self.vias:
            if v is None or v[0] != name: continue
            r = G.VIA_D / 2 + G.CLR + r_sig + MARG + 0.04
            i0, i1, j0, j1 = self.maps.window(v[1] - r, v[2] - r, v[1] + r, v[2] + r)
            X, Y = np.meshgrid(G.xs[i0:i1], G.ys[j0:j1])
            jj, ii = np.nonzero(np.hypot(X - v[1], Y - v[2]) <= r)
            idx = (jj + j0) * G.W + (ii + i0)
            for l in range(4): cells[l].append(idx)
        return {l: np.unique(np.concatenate(c)) for l, c in cells.items() if c}

    def add_territory(self, name):
        with self.lock:
            self.remove_territory(name)
            t = self._terr_cells(name); self.terr[name] = t
            flat = self.occ.reshape(4, -1)
            for l, idx in t.items(): flat[l][idx] += 1

    def remove_territory(self, name):
        with self.lock:
            t = self.terr.pop(name, None)
            if t:
                flat = self.occ.reshape(4, -1)
                for l, idx in t.items(): flat[l][idx] -= 1

    def conflicts(self):
        """Точные конфликты мягких цепей (геометрия меди, а не приближение территорией).
        Возвращает цепи с конфликтами и клетки их осей рядом с местами конфликтов — туда копится история."""
        bad, cells = set(), []
        pts = {}
        for (a, b, l, d, pa, pb) in self.violations(only=self.soft, points=True):
            for n, p in ((a, pa), (b, pb)):
                if n in self.soft:
                    bad.add(n); pts.setdefault(n, []).append(p)
        for n, ps in pts.items():
            for (l, j, i) in self.anchors.get(n, ()):
                x, y = G.xs[i], G.ys[j]
                if any(abs(x - px) < 0.6 and abs(y - py) < 0.6 for px, py in ps): cells.append((l, j, i))
        return bad, cells

    def _congestion(self, win):
        """цена клеток и переходов от чужой территории (PF·занятость) и истории конфликтов"""
        from scipy.ndimage import maximum_filter
        i0, i1, j0, j1 = win
        occ = self.occ[:, j0:j1, i0:i1].astype(np.float32)
        hist = self.hist[:, j0:j1, i0:i1]
        cost = {l: self.pf * occ[l] + hist[l] for l in RL}
        # переход занимает все слои и он шире трассы: территорию расширяем на (VIA_D/2 − ширина/2)
        r = max(1, int(round((G.VIA_D / 2 - self.rules.v['track'] / 2) / RES)))
        yy, xx = np.mgrid[-r:r + 1, -r:r + 1]; fp = (xx * xx + yy * yy) <= r * r
        o = maximum_filter(occ.max(axis=0), footprint=fp, mode='constant')
        via = self.pf * o + hist.max(axis=0)
        return cost, np.ascontiguousarray(via.astype(np.float32))

    # ------------------------------------------------------------------ anchors
    @staticmethod
    def pkey(p): return (p.ref, p.num, round(p.x, 3), round(p.y, 3))

    def pad_cells(self, p):
        """grid cells nearest the pad centre (all layers of the pad)"""
        out = []
        ci, cj = (p.x - G.X0) / RES, (p.y - G.Y0) / RES
        cand = {(int(math.floor(cj)) + a, int(math.floor(ci)) + b) for a in (0, 1) for b in (0, 1)}
        best = sorted(cand, key=lambda c: (c[0] - cj) ** 2 + (c[1] - ci) ** 2)
        d0 = (best[0][0] - cj) ** 2 + (best[0][1] - ci) ** 2
        best = [c for c in best if (c[0] - cj) ** 2 + (c[1] - ci) ** 2 <= d0 + 1e-6]
        for l in p.layers:
            for (j, i) in best: out.append((CU.index(l), j, i))
        return out

    def pad_anchor(self, p):
        return self.pad_cells(p) + self.fan.get(self.pkey(p), [])

    # ------------------------------------------------------------------ search
    def search(self, nid, hl, src, dst, margin, via_target=None, avoid=None, neck=None, lane=None, pair=None,
               maxwin=None, zone=None):
        """hl: {layer: half width or None}; neck: (half width, [(x, y, r), ...]) — сужение у узких площадок;
        lane: callable(win) -> {layer: cost array} для шин и пар; pair: (nid напарника, зазор s, callable(win) -> dist).
        maxwin: (x0, y0, x1, y1) — окно не больше этого (параллельный режим).
        zone: ({слой: полуширина}, [(x, y, r)]) — эти слои разрешены только внутри кругов (быстрые цепи у своих площадок)."""
        pts = src + (dst or [])
        x0 = min(G.xs[c[2]] for c in pts) - margin; x1 = max(G.xs[c[2]] for c in pts) + margin
        y0 = min(G.ys[c[1]] for c in pts) - margin; y1 = max(G.ys[c[1]] for c in pts) + margin
        if maxwin is not None:
            x0, y0 = max(x0, maxwin[0]), max(y0, maxwin[1]); x1, y1 = min(x1, maxwin[2]), min(y1, maxwin[3])
        win = self.maps.window(x0, y0, x1, y1); i0, i1, j0, j1 = win
        w, hh = i1 - i0, j1 - j0
        if w <= 0 or hh <= 0: return None
        nm = None
        if neck and neck[1]:
            Xg, Yg = np.meshgrid(G.xs[i0:i1], G.ys[j0:j1]); nm = np.zeros((hh, w), bool)
            for (px, py, r) in neck[1]:
                nm |= (np.abs(Xg - px) <= r) & (np.abs(Yg - py) <= r)
        pr = None
        if pair is not None:
            pr = (pair[0], pair[1], pair[2](win))
        A, WIDE = {}, {}
        for l in RL:
            h = hl.get(l)
            if h is None:
                A[l] = np.zeros((hh, w), bool); WIDE[l] = A[l]; continue
            a = self.allowed1(nid, l, h, win, pr); WIDE[l] = a
            if nm is not None and neck[0] < h:
                a = a | (self.allowed1(nid, l, neck[0], win, pr) & nm)
            A[l] = a
        if zone:
            Xg, Yg = np.meshgrid(G.xs[i0:i1], G.ys[j0:j1]); zm = np.zeros((hh, w), bool)
            for (px, py, r) in zone[1]:
                zm |= (Xg - px) ** 2 + (Yg - py) ** 2 <= r * r
            for l, h in zone[0].items():
                A[l] = self.allowed1(nid, l, h, win, pr) & zm; WIDE[l] = A[l]
        V = np.ones((hh, w), bool)
        for l in range(4): V &= self.allowed1(nid, l, G.VIA_D / 2, win, pr if l in RL else None)
        if avoid is not None:
            V &= avoid[j0:j1, i0:i1]
        allow = np.stack([A[l] for l in RL]).astype(np.uint8)
        T = np.zeros_like(allow)
        if via_target is not None:
            T[:] = (V & via_target[j0:j1, i0:i1])[None, :, :] & allow.astype(bool)
        lidx = {l: k for k, l in enumerate(RL)}
        S = []
        for (l, j, i) in src:
            if l in lidx and j0 <= j < j1 and i0 <= i < i1:
                k = lidx[l]
                if allow[k, j - j0, i - i0]: S += [k, j - j0, i - i0]
        if dst:
            for (l, j, i) in dst:
                if l in lidx and j0 <= j < j1 and i0 <= i < i1:
                    k = lidx[l]
                    if allow[k, j - j0, i - i0]: T[k, j - j0, i - i0] = 1
        if not S or not T.any(): return None
        kk, jj, ii = np.nonzero(T)
        tx0, tx1, ty0, ty1 = int(ii.min()), int(ii.max()), int(jj.min()), int(jj.max())
        dc = np.array([DIRC[l] for l in RL], np.float32).ravel()
        cost = None; vcost = None; cm = {}
        if lane is not None:
            cm = lane(win) or {}
        congested = bool(self.soft) and self.pf > 0 and nid in {self.netid[n] for n in self.soft}
        if congested:
            cg, vcost = self._congestion(win)
            for l in RL: cm[l] = cm.get(l, 0) + cg[l]
        if ESC_COST > 0:
            for l in RL:
                e = self.esc[l, j0:j1, i0:i1]
                m = (e != 0) & (e != nid)
                if m.any(): cm[l] = cm.get(l, 0) + ESC_COST * m
        if cm:
            cost = np.ascontiguousarray(np.stack([np.broadcast_to(cm.get(l, 0), (hh, w)) for l in RL]).astype(np.float32))
        allow = np.ascontiguousarray(allow); T = np.ascontiguousarray(T)
        Vu = np.ascontiguousarray(V.astype(np.uint8))
        Sa = np.array(S, np.int32)
        cap = 200000; out = np.zeros(cap * 3, np.int32)
        n = _lib.astar(w, hh, len(RL), allow.ctypes.data_as(P_u8), Vu.ctypes.data_as(P_u8), T.ctypes.data_as(P_u8),
                       len(S) // 3, Sa.ctypes.data_as(P_i), dc.ctypes.data_as(P_f), B45, B90, VIAC,
                       tx0, tx1, ty0, ty1, 20_000_000, out.ctypes.data_as(P_i), cap,
                       cost.ctypes.data if cost is not None else None, LANE_HW if cost is not None else 1.0,
                       vcost.ctypes.data if vcost is not None else None)
        if n <= 0: return None
        out = out[:n * 3].reshape(n, 3)
        occw = self.occ[:, j0:j1, i0:i1] if congested else None
        self._ctx = dict(win=win, allow=A, wide=WIDE, cost=None if cost is None else {l: cost[k] for k, l in enumerate(RL)},
                         nid=nid, occ=occw)
        return [(RL[int(k)], int(j) + j0, int(i) + i0) for k, j, i in out]

    # ------------------------------------------------------------------ geometry helpers
    def _raster(self, a, b):
        """cells of an octilinear grid segment a=(j,i) -> b, inclusive"""
        dj, di = b[0] - a[0], b[1] - a[1]
        n = max(abs(dj), abs(di))
        if n == 0: return [a]
        sj, si = _sgn(dj), _sgn(di)
        return [(a[0] + k * sj, a[1] + k * si) for k in range(n + 1)]

    def _free(self, l, a, b):
        """segment a->b on layer l stays inside the allowed area of the last search
        (и в режиме PathFinder не заходит на чужую территорию, куда не заходил исходный путь)"""
        ctx = self._ctx; i0, i1, j0, j1 = ctx['win']; M = ctx['allow'][l]
        occ = ctx.get('occ'); own = ctx.get('own', ())
        cells = self._raster(a, b)
        diag = a[0] != b[0] and a[1] != b[1]
        for (j, i) in cells:
            if not (j0 <= j < j1 and i0 <= i < i1) or not M[j - j0, i - i0]: return False
            if occ is not None and occ[l, j - j0, i - i0] > 0 and (j, i) not in own: return False
        if diag:
            sj, si = _sgn(b[0] - a[0]), _sgn(b[1] - a[1])
            for (j, i) in cells[:-1]:
                side = [(jj, ii) for (jj, ii) in ((j + sj, i), (j, i + si)) if j0 <= jj < j1 and i0 <= ii < i1]
                if not any(M[jj - j0, ii - i0] for (jj, ii) in side): return False
        return True

    def _seg_cost(self, l, a, b):
        n = max(abs(b[0] - a[0]), abs(b[1] - a[1]))
        diag = a[0] != b[0] and a[1] != b[1]
        c = n * (math.sqrt(2) if diag else 1.0)
        cm = self._ctx.get('cost')
        if cm is not None and n:
            i0, i1, j0, j1 = self._ctx['win']; C = cm[l]
            c += sum(float(C[j - j0, i - i0]) for (j, i) in self._raster(a, b)[1:]) * (math.sqrt(2) if diag else 1.0)
        return c

    def _poly_cost(self, l, pts):
        """длина + изломы (90° вдвое дороже 45°: S-сдвиг под прямым углом заменяется плавным 45°) + полоса шины"""
        c = sum(self._seg_cost(l, a, b) for a, b in zip(pts, pts[1:]))
        for a, b, d in zip(pts, pts[1:], pts[2:]):
            u = (_sgn(b[0] - a[0]), _sgn(b[1] - a[1])); v = (_sgn(d[0] - b[0]), _sgn(d[1] - b[1]))
            if u == v: continue
            k = abs(D8.index(u) - D8.index(v)); k = min(k, 8 - k)
            c += SMOOTH_BEND * (1.0 if k == 1 else 2.2 if k == 2 else 4.0)
        return c

    @staticmethod
    def _octi(a, b):
        dj, di = b[0] - a[0], b[1] - a[1]
        return dj == 0 or di == 0 or abs(dj) == abs(di)

    def _links(self, a, b):
        """1- or 2-segment octilinear connections a -> b"""
        if self._octi(a, b): return [[a, b]]
        dj, di = b[0] - a[0], b[1] - a[1]; sj, si = _sgn(dj), _sgn(di)
        m = min(abs(dj), abs(di))
        if abs(di) > abs(dj):
            return [[a, (a[0], a[1] + si * (abs(di) - m)), b], [a, (a[0] + sj * m, a[1] + si * m), b]]
        return [[a, (a[0] + sj * (abs(dj) - m), a[1]), b], [a, (a[0] + sj * m, a[1] + si * m), b]]

    @staticmethod
    def _corners(run):
        pts = [run[0]]
        for k in range(1, len(run) - 1):
            a, b, c = pts[-1], run[k], run[k + 1]
            d1 = (b[0] - a[0], b[1] - a[1]); d2 = (c[0] - b[0], c[1] - b[1])
            if d1[0] * d2[1] - d1[1] * d2[0] != 0: pts.append(b)
        if len(run) > 1: pts.append(run[-1])
        return pts

    def smooth(self, l, cells):
        """спрямление: жадно заменяет участки ломаной на 1–2 отрезка, если это дешевле (длина + изломы + полоса шины)"""
        self._ctx['own'] = set(cells)
        pts = self._corners(cells)
        if len(pts) <= 2: return pts
        out = [pts[0]]; s = 0
        while s < len(pts) - 1:
            best = None
            for e in range(len(pts) - 1, s + 1, -1):
                old = self._poly_cost(l, pts[s:e + 1])
                for cand in self._links(pts[s], pts[e]):
                    if all(self._free(l, a, b) for a, b in zip(cand, cand[1:])):
                        c = self._poly_cost(l, cand)
                        if c < old - 1e-6 and (best is None or c < best[0]): best = (c, cand, e)
                if best: break
            if best:
                out += best[1][1:]; s = best[2]
            else:
                out.append(pts[s + 1]); s += 1
        return self._corners(out)

    # ------------------------------------------------------------------ commit
    def commit(self, name, hl, path, tag, src_pad=None, dst_pads=(), via_end=False, neck_h=None):
        runs, cur, vias = [], [path[0]], []
        for c in path[1:]:
            if c[0] != cur[-1][0]:
                runs.append(cur); vias.append((c[1], c[2])); cur = [c]
            else: cur.append(c)
        runs.append(cur)
        if via_end: vias.append((path[-1][1], path[-1][2]))
        anchors = self.anchors.setdefault(name, set()) if tag != 'fan' else set()
        newcells = []
        for ri, run in enumerate(runs):
            l = run[0][0]
            grid = self.smooth(l, [(c[1], c[2]) for c in run])
            # widths per grid segment (wide where the full-width map allows, neck-down elsewhere)
            wide = self._ctx['wide'][l]; i0, i1, j0, j1 = self._ctx['win']
            wfull = 2 * hl[l]; wneck = 2 * neck_h if neck_h else wfull
            V, E = [grid[0]], []
            for a, b in zip(grid, grid[1:]):
                cells = self._raster(a, b)
                for c in cells: anchors.add((l,) + c); newcells.append((l,) + c)
                ok = [bool(wide[j - j0, i - i0]) for (j, i) in cells]
                k = 0
                while k < len(cells) - 1:
                    m = k + 1
                    flag = ok[k] and ok[k + 1]
                    while m < len(cells) - 1 and (ok[m] and ok[m + 1]) == flag: m += 1
                    V.append(cells[m]); E.append(wfull if flag else wneck); k = m
            P = [(float(G.xs[i]), float(G.ys[j])) for (j, i) in V]
            starts_pad = src_pad if ri == 0 and src_pad is not None and path[0] in set(self.pad_cells(src_pad)) else None
            end_pad = None
            if ri == len(runs) - 1 and not via_end:
                for q in dst_pads:
                    if path[-1] in set(self.pad_cells(q)): end_pad = q; break
            P, E = self.align(name, l, P, E, starts_pad, end_pad)
            for (a, b), w in zip(zip(P, P[1:]), E):
                self.add_track(name, l, a[0], a[1], b[0], b[1], w, tag)
            if len(run) == 1: anchors.add(run[0]); newcells.append(run[0])
        for (j, i) in vias:
            self.add_via(name, G.xs[i], G.ys[j], tag)
            for l in range(4): anchors.add((l, j, i)); newcells.append((l, j, i))
        return newcells

    # ------------------------------------------------------------------ exact pad entry
    def _clear_ok(self, name, l, a, b, w):
        """консервативная проверка зазора отрезка по картам расстояний (узел сетки ± смещение)"""
        nid = self.netid[name]
        L = math.hypot(b[0] - a[0], b[1] - a[1]); n = max(1, int(L / 0.05))
        for k in range(n + 1):
            x = a[0] + (b[0] - a[0]) * k / n; y = a[1] + (b[1] - a[1]) * k / n
            i, j = G.gx(x), G.gy(y)
            if not (0 <= i < G.W and 0 <= j < G.H): return False
            off = math.hypot(x - G.xs[i], y - G.ys[j])
            if self.maps.n1[l, j, i] == nid: d = self.maps.d2[l, j, i]
            else: d = self.maps.d1[l, j, i]
            if d - off < G.CLR + w / 2 - 1e-3: return False
            if self.maps.edge[j, i] - off < G.EDGE + w / 2 - 1e-3: return False
            if name in self.soft and self.occ[l, j, i] > 0: return False     # мягкие соседи (PathFinder)
        return True

    def _shift_end(self, name, l, P, E, pad, at_start):
        """сдвинуть прямой участок у площадки так, чтобы он проходил точно через её центр"""
        if at_start: Q, F = list(P), list(E)
        else: Q, F = list(reversed(P)), list(reversed(E))
        if len(Q) < 2: return None
        ox, oy = pad.x - Q[0][0], pad.y - Q[0][1]
        dx, dy = Q[1][0] - Q[0][0], Q[1][1] - Q[0][1]
        if abs(dy) < 1e-9 and abs(dx) > 1e-9: ax, perp = 'h', oy
        elif abs(dx) < 1e-9 and abs(dy) > 1e-9: ax, perp = 'v', ox
        else: return None
        def straight(a, b):
            return abs(b[1] - a[1]) < 1e-9 if ax == 'h' else abs(b[0] - a[0]) < 1e-9
        k = 1
        while k + 1 < len(Q) and straight(Q[k], Q[k + 1]): k += 1
        R = [list(q) for q in Q]
        for m in range(0, k + 1):
            if ax == 'h': R[m][1] += perp
            else: R[m][0] += perp
        if ax == 'h': R[0][0] = pad.x
        else: R[0][1] = pad.y
        if k + 1 < len(Q):
            nx, ny = Q[k + 1][0] - Q[k][0], Q[k + 1][1] - Q[k][1]
            if abs(nx) > 1e-9 and abs(ny) > 1e-9:          # keep 45 deg: the bend slides along the following diagonal
                ux, uy = _sgn(nx), _sgn(ny)
                if ax == 'h': R[k] = [Q[k][0] + perp * ux * uy, Q[k][1] + perp]
                else: R[k] = [Q[k][0] + perp, Q[k][1] + perp * ux * uy]
        else:
            return None                                     # the whole run is one straight: far end is fixed
        if math.hypot(R[0][0] - Q[0][0], R[0][1] - Q[0][1]) > 0.2: return None
        for m in range(0, min(k + 1, len(R) - 1)):
            if not self._clear_ok(name, l, tuple(R[m]), tuple(R[m + 1]), F[m]): return None
        R = [tuple(r) for r in R]
        return (R, F) if at_start else (list(reversed(R)), list(reversed(F)))

    def align(self, name, l, P, E, spad, epad):
        """Точный вход в центр площадки сдвигом прямого участка. Если сдвинуть нельзя (зазор, участок до
        перехода), трасса заканчивается в ближайшем к центру узле сетки — он внутри меди площадки
        (≤ 0.07 мм от центра), соединение то же, а косого или ступенчатого хвостика нет."""
        if len(P) < 2: return P, E
        for pad, at_start in ((spad, True), (epad, False)):
            if pad is None: continue
            r = self._shift_end(name, l, P, E, pad, at_start)
            if r: P, E = r
        return P, E

    def add_track(self, name, l, ax, ay, bx, by, w, tag):
        if math.hypot(bx - ax, by - ay) < 1e-6: return
        with self.lock:
            self.tracks.append([name, l, float(ax), float(ay), float(bx), float(by), float(w), tag])
        if name in self.soft: return
        f, bb = seg_dist(ax, ay, bx, by, w); self.maps.add(l, self.netid[name], bb, f)

    def add_via(self, name, x, y, tag):
        with self.lock:
            self.vias.append([name, float(x), float(y), tag])
        if name in self.soft: return
        f, bb = circ_dist(x, y, G.VIA_D / 2)
        for l in range(4): self.maps.add(l, self.netid[name], bb, f)

    # ------------------------------------------------------------------ fanout
    def fan_width(self, p):
        st = self.rules.widths(p.net, self.span(p.net))[0]
        vals = [v for v in st.values() if v]
        w = min(vals) if vals else self.rules.v['track']
        if p.net in self.rules.pair: w = self.rules.pair[p.net][1]['w']
        # отвод короткий (площадка → переход), поэтому по току его можно сузить до ширины площадки − 0.05:
        # иначе между соседними выводами с шагом 0.5 мм он не проходит по зазору
        return max(self.rules.v['track_min'], min(w, min(p.w, p.h) - 0.05))

    def fanout(self, ref, nums=None, margin=3.0):
        """Вывести площадки микросхемы на переходы. Глубина перехода (у края площадки или на ряд дальше)
        выбирается для каждого вывода по загрузке: дальше от уже поставленных переходов соседей —
        значит, между ними остаётся канал под трассу."""
        ps = [p for p in self.pads if p.ref == ref and p.net and p.kind == 'smd'
              and min(p.w, p.h) <= 1.0 and (nums is None or p.num in nums)]
        # обход по периметру корпуса: соседние выводы — подряд, чтобы выбор глубины учитывал соседа
        if ps:
            cx = sum(p.x for p in ps) / len(ps); cy = sum(p.y for p in ps) / len(ps)
            ps.sort(key=lambda p: math.atan2(p.y - cy, p.x - cx))
        bad = []
        for p in ps:
            if not self.fan_one(p, margin): bad.append(p)
        still = []
        for p in bad:
            neigh = sorted([q for q in self.pads if q.ref == p.ref and q is not p and self.pkey(q) in self.fanrec
                            and math.hypot(q.x - p.x, q.y - p.y) < 1.6], key=lambda q: math.hypot(q.x - p.x, q.y - p.y))
            fixed = False
            for q in neigh:
                saved = ([list(t) if t else None for t in self.tracks], [list(v) if v else None for v in self.vias], dict(self.fan), dict(self.fanrec))
                self.unfan(q); self.rebuild()
                if self.fan_one(p, margin + 1) and self.fan_one(q, margin + 1):
                    fixed = True; break
                self.tracks, self.vias, self.fan, self.fanrec = saved; self.rebuild()
            if not fixed: still.append(p.num)
        return still

    def unfan(self, q):
        t0, t1, v0, v1 = self.fanrec.pop(self.pkey(q))
        for k in range(t0, t1): self.tracks[k] = None
        for k in range(v0, v1): self.vias[k] = None
        self.fan.pop(self.pkey(q), None)

    def fan_one(self, p, margin):
        w = self.fan_width(p); h = w / 2
        hl = {l: h for l in RL}
        f, _ = pad_shape(p)
        i0, i1, j0, j1 = self.maps.window(p.x - margin, p.y - margin, p.x + margin, p.y + margin)
        Xg, Yg = np.meshgrid(G.xs[i0:i1], G.ys[j0:j1])
        sub = f(Xg, Yg)
        fanvias = [(v[1], v[2]) for v in self.vias if v is not None and v[3] == 'fan'
                   and abs(v[1] - p.x) < 4 and abs(v[2] - p.y) < 4]
        best = None
        deeps = (0.0, 1.0, 1.8) if min(p.w, p.h) <= 0.35 else (0.0,)
        for deep in deeps:
            vt = np.zeros((G.H, G.W), bool)
            vt[j0:j1, i0:i1] = sub >= deep
            path = self.search(p.nid, hl, self.pad_cells(p), None, margin, via_target=vt)
            if not path: continue
            ex, ey = G.xs[path[-1][2]], G.ys[path[-1][1]]
            gap = min([math.hypot(ex - x, ey - y) for x, y in fanvias] + [9.0])
            # канал между переходами: шаг переходов ≥ VIA_D + 2·зазор + трасса — иначе соседний ряд
            need = G.VIA_D + 2 * G.CLR + self.rules.v['track']
            score = (min(gap, need), -len(path))
            if best is None or score > best[0]: best = (score, path, dict(self._ctx))
            if gap >= need: break
        if best is None: return False
        _, path, ctx = best
        self._ctx = ctx
        t0, v0 = len(self.tracks), len(self.vias)
        cells = self.commit(p.net, hl, path, 'fan', src_pad=p, via_end=True)
        self.fan[self.pkey(p)] = cells
        self.fanrec[self.pkey(p)] = (t0, len(self.tracks), v0, len(self.vias))
        return True

    # ------------------------------------------------------------------ nets
    def span(self, name):
        ps = [p for p in self.pads if p.net == name and p.kind != 'np_thru_hole']
        if not ps: return 0.0
        return (max(p.x for p in ps) - min(p.x for p in ps)) + (max(p.y for p in ps) - min(p.y for p in ps))

    def steps(self, name):
        st = self.rules.widths(name, self.span(name))
        if name in self.rules.pair:
            info = self.rules.pair[name][1]
            st = [{l: (info['w'] if l in info['layers'][:1] else None) for l in RL}]
        return st

    def _neck(self, pads, wmax):
        """площадки, к которым трасса полной ширины не подойдёт: узкие выводы микросхем"""
        out = []
        for p in pads:
            if min(p.w, p.h) < wmax + 0.05:
                out.append((p.x, p.y, max(p.w, p.h) / 2 + NECK_R))
                # переходы fanout этой площадки стоят в плотном поле выводов — там тоже можно сужаться
                for (l, j, i) in self.fan.get(self.pkey(p), []):
                    out.append((float(G.xs[i]), float(G.ys[j]), NECK_R))
        # у площадки трасса не шире площадки − 0.05 (как отвод fanout): иначе при шаге 0.5 мм не проходит по зазору
        nh = max(self.rules.v['track_min'], min([min(p.w, p.h) - 0.05 for p in pads] + [wmax])) / 2
        return nh, out

    def _lane_fn(self, sib, hl, cost, rr, pitch_of):
        def fn(win):
            i0, i1, j0, j1 = win
            out = {}
            X = G.xs[i0:i1]; Y = G.ys[j0:j1]
            for l in RL:
                C = np.full((j1 - j0, i1 - i0), cost, np.float32)
                h = hl.get(l)
                if h is None: out[l] = C; continue
                for t in sib:
                    if t[1] != l: continue
                    pitch = pitch_of(h, t)
                    R = pitch + rr
                    bx0, bx1 = min(t[2], t[4]) - R, max(t[2], t[4]) + R
                    by0, by1 = min(t[3], t[5]) - R, max(t[3], t[5]) + R
                    a0 = max(0, int(np.searchsorted(X, bx0))); a1 = min(len(X), int(np.searchsorted(X, bx1)) + 1)
                    b0 = max(0, int(np.searchsorted(Y, by0))); b1 = min(len(Y), int(np.searchsorted(Y, by1)) + 1)
                    if a0 >= a1 or b0 >= b1: continue
                    Xg, Yg = np.meshgrid(X[a0:a1], Y[b0:b1])
                    f, _ = seg_dist(t[2], t[3], t[4], t[5], 0.0)
                    d = f(Xg, Yg)
                    c = cost * np.clip(np.abs(d - pitch) / rr, 0, 1)
                    C[b0:b1, a0:a1] = np.minimum(C[b0:b1, a0:a1], c.astype(np.float32))
                out[l] = C
            return out
        return fn

    def _lane(self, name, hl):
        """карта стоимости полосы: шина — на шаге «ширина + зазор» от соседей; дифпара — на шаге w + s от напарника"""
        if name in self.rules.pair:
            partner, info = self.rules.pair[name]
            sib = [t for t in self.tracks if t is not None and t[0] == partner and t[7] != 'fan']
            if not sib: return None
            s = info['s']
            return self._lane_fn(sib, hl, PAIR_COST, 0.6, lambda h, t: h + s + t[6] / 2)
        g = self.group_of.get(name)
        if not g: return None
        sib = [t for t in self.tracks if t is not None and t[0] != name and t[0] in self.groups[g] and t[7] != 'fan']
        if not sib: return None
        # шаг полосы — сразу за границей территории соседа (иначе полоса тянет цепь в конфликт с PathFinder)
        return self._lane_fn(sib, hl, LANE_COST, LANE_R, lambda h, t: h + G.CLR + t[6] / 2 + 0.06)

    def _pair_arg(self, name):
        """для второго проводника пары: зазор s до напарника вместо обычного"""
        if name not in self.rules.pair: return None
        partner, info = self.rules.pair[name]
        if not any(t is not None and t[0] == partner for t in self.tracks): return None
        pnid = self.netid[partner]
        def dist(win):
            i0, i1, j0, j1 = win
            X, Y = np.meshgrid(G.xs[i0:i1], G.ys[j0:j1]); out = {}
            for l in RL:
                D = np.full(X.shape, CAP, np.float32)
                for t in self.tracks:
                    if t is None or t[0] != partner or t[1] != l: continue
                    if max(t[2], t[4]) + CAP < X[0, 0] or min(t[2], t[4]) - CAP > X[0, -1]: continue
                    f, _ = seg_dist(t[2], t[3], t[4], t[5], t[6]); D = np.minimum(D, f(X, Y))
                for v in self.vias:
                    if v is None or v[0] != partner: continue
                    D = np.minimum(D, np.hypot(X - v[1], Y - v[2]) - G.VIA_D / 2)
                for p in self.pads:
                    if p.net == partner and CU[l] in p.layers and abs(p.x - X[0, 0]) < 1e3:
                        f, _ = pad_shape(p); D = np.minimum(D, f(X, Y))
                out[l] = D
            return out
        return (pnid, info['s'], dist)

    def _mst(self, pads, root):
        """Prim: порядок подключения и родитель каждой площадки"""
        order, parent = [root], {id(root): None}
        rest = [p for p in pads if p is not root]
        best = {id(p): (octile(p.x, p.y, root.x, root.y), root) for p in rest}
        while rest:
            p = min(rest, key=lambda q: best[id(q)][0])
            rest.remove(p); order.append(p); parent[id(p)] = best[id(p)][1]
            for q in rest:
                d = octile(p.x, p.y, q.x, q.y)
                if d < best[id(q)][0]: best[id(q)] = (d, p)
        return order, parent

    def _root(self, pads):
        """корень дерева — вывод «ведущей» микросхемы (ref с наибольшим числом выводов на плате)"""
        cnt = {}
        for p in self.pads: cnt[p.ref] = cnt.get(p.ref, 0) + 1
        ics = [p for p in pads if p.ref[:1] == 'U']
        if ics: return max(ics, key=lambda p: (cnt[p.ref], -p.x))
        return min(pads, key=lambda p: (p.x, p.y))

    def route_gnd(self, name='GND'):
        nid = self.netid[name]
        pads = [p for p in self.pads if p.net == name and p.kind == 'smd']
        ws = self.rules.v['gnd_stub']
        for p in sorted(pads, key=lambda p: (p.x, p.y)):
            if self.pkey(p) in self.fan: continue
            if min(p.w, p.h) > 2.0: continue          # exposed pad: thermal vias already
            if set(self.pad_cells(p)) & self.anchors.get(name, set()): continue   # уже подключена
            ok = False
            for w in ws:
                w = min(w, max(self.rules.v['track_min'], min(p.w, p.h)))
                hl = {l: w / 2 for l in RL}
                for reach in (1.5, 2.5, 5.0):         # переход на плоскость — как можно ближе к площадке
                    tree = self.anchors.get(name, set())
                    near = [c for c in tree if abs(G.xs[c[2]] - p.x) < reach and abs(G.ys[c[1]] - p.y) < reach]
                    vt = np.ones((G.H, G.W), bool)
                    path = self.search(nid, hl, self.pad_cells(p), near or None, reach, via_target=vt)
                    if path:
                        end = path[-1]
                        on_tree = end in tree
                        self.commit(name, hl, path, 'gnd', src_pad=p, via_end=not on_tree)
                        ok = True; break
                if ok: break
            if not ok: self.fail.append((name, p.ref, p.num))

    def route_net(self, name, steps=None, gnd=False, maxwin=None):
        """Развести цепь. maxwin — не выходить за окно (параллельный режим): возвращает False,
        если внутри окна пути нет и цепь надо развести позже, без ограничения."""
        if gnd: return self.route_gnd(name)
        nid = self.netid[name]
        pads = [p for p in self.pads if p.net == name and p.kind != 'np_thru_hole']
        if len(pads) < 2: return True
        if steps is None: steps = self.steps(name)
        root = self._root(pads)
        order, parent = self._mst(pads, root)
        tree = set(self.pad_anchor(root)) | self.anchors.get(name, set())
        conn = [root]
        pair = self._pair_arg(name)
        fails = []
        # быстрая цепь: «чужие» слои (без опорной плоскости) — только короткий выход у своих площадок,
        # например перескок D+ через D− под USB-C
        fast_zone = None
        if self.rules.is_fast(name):
            wz = self.rules.pair[name][1]['w'] if name in self.rules.pair else self.rules.v['track']
            fast_zone = ({l: wz / 2 for l in RL if l not in self.rules.fast_layers()},
                         [(q.x, q.y, 1.5) for q in pads] +
                         [(float(G.xs[c[2]]), float(G.ys[c[1]]), 1.0) for q in pads for c in self.fan.get(self.pkey(q), [])])
        for p in order[1:]:
            q = parent[id(p)]
            ok = False
            for si, st in enumerate(steps):
                hl = {l: (st[l] / 2 if st.get(l) else None) for l in RL}
                if not any(hl.values()): continue
                wmax = max(v for v in st.values() if v)
                neck = self._neck([p] + [x for x in conn], wmax)
                lane = self._lane(name, hl)
                path = None
                # 1) к своему соседу по дереву (и трассам рядом с ним), 2) к любой точке цепи;
                # шина/пара сначала ищет путь по своей полосе, потом — без неё
                bx0, bx1 = min(p.x, q.x) - 2.0, max(p.x, q.x) + 2.0
                by0, by1 = min(p.y, q.y) - 2.0, max(p.y, q.y) + 2.0
                local = set(self.pad_anchor(q)) | {c for c in tree if bx0 <= G.xs[c[2]] <= bx1 and by0 <= G.ys[c[1]] <= by1}
                tries = [(lane, local, 3.0), (lane, local, 8.0), (lane, tree, 8.0), (lane, tree, 25.0)] if lane else []
                tries += [(None, local, 3.0), (None, local, 8.0), (None, tree, 8.0), (None, tree, 25.0), (None, tree, 80.0)]
                for ln, dst, m in tries:
                    path = self.search(nid, hl, self.pad_anchor(p), sorted(dst), m, neck=neck, lane=ln, pair=pair,
                                       maxwin=maxwin, zone=fast_zone)
                    if path: break
                if path:
                    end = path[-1]
                    dst = [x for x in conn if end in set(self.pad_cells(x))]
                    hlc = dict(hl)
                    if fast_zone: hlc.update({l: h for l, h in fast_zone[0].items() if hlc.get(l) is None})
                    cells = self.commit(name, hlc, path, 'net', src_pad=p if path[0] in set(self.pad_cells(p)) else None,
                                        dst_pads=dst, neck_h=neck[0])
                    tree |= set(cells) | set(self.pad_anchor(p))
                    conn.append(p); ok = True
                    if si > 0 and si == len(steps) - 1 and self.rules.current(name) > 0:
                        self.warn.append((name, p.ref, p.num, 'сигнальная ширина: ниже расчётной по току'))
                    elif si > 0 and self.rules.current(name) > 0:
                        self.warn.append((name, p.ref, p.num, 'ширина по нагреву, без запаса на падение напряжения'))
                    break
            if not ok:
                fails.append((name, p.ref, p.num))
                tree |= set(self.pad_anchor(p)); conn.append(p)
        if maxwin is not None and fails:
            return False
        self.fail += fails
        return True

    # ------------------------------------------------------------------ PathFinder
    def negotiate(self, nets, iters=30, log=print, threads=1):
        """Согласованная перегрузка: все цепи из nets разводятся «мягко» — за чужую территорию платят
        PF·занятость + история. После каждого прохода PF растёт, в клетках конфликтов копится история;
        переразводятся только цепи с конфликтами. Возвращает число проходов и оставшиеся конфликты."""
        import time
        nets = [n for n in nets if n in self.netid]
        for n in nets: self.rip(n)
        self.soft = set(nets); self.rebuild()
        self.pf = PF0
        todo = list(nets); bad = set(); it = 0; best = None; stall = 0
        for it in range(1, iters + 1):
            t0 = time.time()
            self._route_batch(todo, threads)
            bad, cells = self.conflicts()
            for (l, j, i) in cells: self.hist[l, j, i] += HINC
            log('  PathFinder проход %d: развели %d, конфликтов у %d цепей (PF=%.2f) %.1f с'
                % (it, len(todo), len(bad), self.pf, time.time() - t0))
            if not bad: break
            # нет улучшения 5 проходов подряд — остаток доделает узаконивание (точный rip-up)
            if best is None or len(bad) < best: best, stall = len(bad), 0
            else: stall += 1
            if stall >= 5: break
            self.pf = min(self.pf * PF_MUL, 50.0)
            todo = [n for n in nets if n in bad]
        # узаконить: все цепи — снова жёсткие препятствия
        self.soft = set(); self.pf = 0.0; self.rebuild()
        return it, bad

    def _route_batch(self, todo, threads):
        if threads <= 1 or len(todo) < 2:
            for n in todo:
                self.rip(n); self.route_net(n); self.add_territory(n)
            return
        # параллельно — цепи, окна которых (bbox + 8 мм + радиус влияния) не пересекаются
        from concurrent.futures import ThreadPoolExecutor
        M = 8.0 + CAP + 1.0
        def box(n):
            ps = [p for p in self.pads if p.net == n]
            return (min(p.x for p in ps) - M, min(p.y for p in ps) - M, max(p.x for p in ps) + M, max(p.y for p in ps) + M)
        rest = list(todo); later = []
        with ThreadPoolExecutor(threads) as ex:
            while rest:
                batch, boxes = [], []
                for n in list(rest):
                    if n in self.group_of or n in self.rules.pair:     # полосам нужны соседи — по одному
                        if batch: continue
                    b = box(n)
                    if all(b[2] < c[0] or c[2] < b[0] or b[3] < c[1] or c[3] < b[1] for c in boxes):
                        batch.append(n); boxes.append(b); rest.remove(n)
                        if n in self.group_of or n in self.rules.pair: break
                for n in batch: self.rip(n)
                if len(batch) == 1:
                    self.route_net(batch[0]); self.add_territory(batch[0]); continue
                res = list(ex.map(lambda nb: self.route_net(nb[0], maxwin=nb[1]), zip(batch, boxes)))
                for n, ok in zip(batch, res):
                    if ok: self.add_territory(n)
                    else: self.rip(n); later.append(n)
        for n in later:
            self.rip(n); self.route_net(n); self.add_territory(n)

    # ------------------------------------------------------------------ legalize
    def violations(self, only=None, points=False):
        """точная проверка зазоров между разными цепями (shapely): [(цепь_a, цепь_b, слой, зазор)];
        only — проверять только пары, где хотя бы одна цепь из этого множества; points — добавить точки конфликта"""
        from shapely.geometry import LineString, Point
        from shapely.strtree import STRtree
        from shapely.ops import nearest_points
        out = []
        for l in range(4):
            items = []
            for t in self.tracks:
                if t is None or t[1] != l: continue
                items.append((LineString([(t[2], t[3]), (t[4], t[5])]).buffer(t[6] / 2, 8), t[0], t[7]))
            for v in self.vias:
                if v is None: continue
                items.append((Point(v[1], v[2]).buffer(G.VIA_D / 2, 16), v[0], v[3]))
            if not items: continue
            geoms = [g for g, _, _ in items]; tree = STRtree(geoms)
            for k, (g, n, tag) in enumerate(items):
                for j in tree.query(g.buffer(G.CLR)):
                    if j <= k: continue
                    g2, n2, _ = items[j]
                    if n2 == n: continue
                    if only is not None and n not in only and n2 not in only: continue
                    lim = G.CLR
                    if n in self.rules.pair and self.rules.pair[n][0] == n2: lim = self.rules.pair[n][1]['s']
                    d = g.distance(g2)
                    if d < lim - 1e-3:
                        if points:
                            a, b = nearest_points(g, g2)
                            out.append((n, n2, l, round(d, 3), (a.x, a.y), (b.x, b.y)))
                        else:
                            out.append((n, n2, l, round(d, 3)))
        return out

    # ------------------------------------------------------------------ rip-up
    def rip(self, name):
        self.remove_territory(name)
        hard = name not in self.soft
        for k, t in enumerate(self.tracks):
            if t is not None and t[0] == name and t[7] != 'fan' and t[7] != 'fixed': self.tracks[k] = None
        for k, v in enumerate(self.vias):
            if v is not None and v[0] == name and v[3] != 'fan' and v[3] != 'fixed': self.vias[k] = None
        keep = set()
        for v in self.vias:
            if v is not None and v[0] == name and v[3] == 'fixed':
                for l in range(4): keep.add((l, G.gy(v[2]), G.gx(v[1])))
        self.anchors[name] = keep
        self.fail = [f for f in self.fail if f[0] != name]
        self.warn = [f for f in self.warn if f[0] != name]
        return hard

    def split(self, name, pad):
        """Разделить медь цепи на связную часть с площадкой pad (A) и всё остальное.
        Возвращает (якоря A, якоря остального) — именно между ними и нужен путь."""
        from shapely.geometry import Point, LineString
        from shapely.ops import unary_union
        from post import padpoly
        items = []
        pads = [p for p in self.pads if p.net == name and p.kind != 'np_thru_hole']
        for p in pads:
            g = padpoly(p)
            for l in p.layers: items.append((CU.index(l), g, ('p', id(p))))
        for k, t in enumerate(self.tracks):
            if t is None or t[0] != name: continue
            g = LineString([(t[2], t[3]), (t[4], t[5])]).buffer(t[6] / 2, 4)
            items.append((t[1], g, ('t', k)))
        for k, v in enumerate(self.vias):
            if v is None or v[0] != name: continue
            g = Point(v[1], v[2]).buffer(G.VIA_D / 2, 8)
            for l in range(4): items.append((l, g, ('v', k)))
        par = list(range(len(items)))
        def f(a):
            while par[a] != a: par[a] = par[par[a]]; a = par[a]
            return a
        bykey = {}
        for i, it in enumerate(items): bykey.setdefault(it[2], []).append(i)
        for ids in bykey.values():
            for i in ids[1:]: par[f(i)] = f(ids[0])
        for l in range(4):
            idx = [i for i, it in enumerate(items) if it[0] == l]
            for a in range(len(idx)):
                for b in range(a + 1, len(idx)):
                    i, j = idx[a], idx[b]
                    if f(i) != f(j) and items[i][1].intersects(items[j][1]): par[f(i)] = f(j)
        root = f(bykey[('p', id(pad))][0])
        inA = [f(i) == root for i in range(len(items))]
        geoA = {l: unary_union([it[1] for i, it in enumerate(items) if inA[i] and it[0] == l] or [Point(0, 0).buffer(0)])
                for l in range(4)}
        A, B = set(), set()
        for p in pads:
            (A if f(bykey[('p', id(p))][0]) == root else B).update(self.pad_anchor(p))
        for c in self.anchors.get(name, set()):
            if c in A: continue
            pt = Point(float(G.xs[c[2]]), float(G.ys[c[1]]))
            (A if geoA[c[0]].distance(pt) < 0.06 else B).add(c)
        return A, B - A

    def blockers(self, name, pad, steps=None):
        """route ignoring other nets' tracks; return nets whose tracks/vias collide with that path"""
        from shapely.geometry import LineString, Point
        if steps is None: steps = self.steps(name)
        st = steps[-1]; hl = {l: (st[l] / 2 if st.get(l) else None) for l in RL}
        h = max(v for v in hl.values() if v)
        saved = self.maps
        srcA, tree = self.split(name, pad)          # путь нужен между частями цепи, а не внутри своей
        # «мягкая» карта: только площадки, своя цепь и fanout/фиксированные переходы
        self.maps = Maps(); self.build_pads()
        for t in self.tracks:
            if t is None: continue
            if t[0] == name or t[7] in ('fan', 'fixed'):
                f, bb = seg_dist(t[2], t[3], t[4], t[5], t[6]); self.maps.add(t[1], self.netid[t[0]], bb, f)
        for v in self.vias:
            if v is None: continue
            if v[0] == name or v[3] in ('fan', 'fixed'):
                f, bb = circ_dist(v[1], v[2], G.VIA_D / 2)
                for l in range(4): self.maps.add(l, self.netid[v[0]], bb, f)
        if not tree:
            self.maps = saved; return {}
        neck = self._neck([q for q in self.pads if q.net == name], 2 * h)
        path = None
        for m in (3.0, 8.0, 25.0, 80.0):
            path = self.search(self.netid[name], hl, sorted(srcA), sorted(tree), m, neck=neck)
            if path: break
        self.maps = saved
        if not path: return None
        hit = {}
        byl = {}
        for (l, j, i) in path: byl.setdefault(l, []).append((G.xs[i], G.ys[j]))
        geo = {l: (LineString(p) if len(p) > 1 else Point(p[0])) for l, p in byl.items()}
        vpts = [Point(G.xs[a[2]], G.ys[a[1]]) for a, b in zip(path, path[1:]) if a[0] != b[0]]
        for t in self.tracks:
            if t is None or t[0] == name or t[7] in ('fan', 'fixed') or not vpts: continue
            seg = LineString([(t[2], t[3]), (t[4], t[5])])
            if any(seg.distance(vp) < G.CLR + G.VIA_D / 2 + t[6] / 2 + 0.02 for vp in vpts):
                hit[t[0]] = hit.get(t[0], 0) + 1
        for t in self.tracks:
            if t is None or t[0] == name or t[7] in ('fan', 'fixed'): continue
            g = geo.get(t[1])
            if g is not None and g.distance(LineString([(t[2], t[3]), (t[4], t[5])])) < G.CLR + h + t[6] / 2 + 0.02:
                hit[t[0]] = hit.get(t[0], 0) + 1
        for v in self.vias:
            if v is None or v[0] == name or v[3] in ('fan', 'fixed'): continue
            if any(g.distance(Point(v[1], v[2])) < G.CLR + h + G.VIA_D / 2 + 0.02 for g in geo.values()):
                hit[v[0]] = hit.get(v[0], 0) + 1
        return hit

    # ------------------------------------------------------------------ output
    def export(self):
        tr = [tuple(t[:7]) for t in self.tracks if t is not None]
        vi = [tuple(v[:3]) for v in self.vias if v is not None]
        return tr, vi

    def emit(self, src, dst, tracks, vias, zones=()):
        txt = open(src, encoding='utf-8').read(); out = []
        for (name, l, ax, ay, bx, by, w) in tracks:
            if math.hypot(bx - ax, by - ay) < 1e-6: continue
            out.append('\t(segment\n\t\t(start %.4f %.4f)\n\t\t(end %.4f %.4f)\n\t\t(width %.3f)\n\t\t(layer "%s")\n\t\t(net "%s")\n\t\t(uuid "%s")\n\t)' % (ax, ay, bx, by, w, CU[l], name, uuid.uuid4()))
        for (name, x, y) in vias:
            out.append('\t(via\n\t\t(at %.4f %.4f)\n\t\t(size %.2f)\n\t\t(drill %.2f)\n\t\t(layers "F.Cu" "B.Cu")\n\t\t(net "%s")\n\t\t(uuid "%s")\n\t)' % (x, y, G.VIA_D, G.VIA_DRILL, name, uuid.uuid4()))
        out += list(zones)
        # вставить перед первой зоной верхнего уровня (не «zone_connect» внутри площадок), иначе — перед концом файла
        m = re.search(r'\n\t\(zone\b', txt)
        if m: k = m.start() + 1
        else:
            m = re.search(r'\n\t\(embedded_fonts', txt)
            k = m.start() + 1 if m else txt.rstrip().rindex(')')
        open(dst, 'w', encoding='utf-8').write(txt[:k] + '\n'.join(out) + '\n' + txt[k:])
