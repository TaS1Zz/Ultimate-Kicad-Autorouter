"""Полигоны питания и расчёт падения напряжения по меди.

zones(): цепям с током от power.polygon_current А добавляется зона (полигон) на внешнем слое вдоль уже
проложенной трассы — шире на polygon_margin с каждой стороны. Заливает её KiCad (kicad-cli --refill-zones)
с обычными зазорами до чужой меди; приоритет выше заливки GND на том же слое.

ir_drop(): падение напряжения по реальной меди цепи после заливки. Медь каждого слоя растрируется с шагом
0.1 мм в сетку резисторов (сопротивление квадрата ρ/t), слои соединены переходами и штыревыми площадками
(стенка металлизации 25 мкм), площадки источника — 0 В, в площадки потребителей втекает их ток.
Это решение уравнения Лапласа для листовой меди — точнее оценки «весь ток через все отрезки».
"""
import math, uuid
import numpy as np
from shapely.geometry import LineString, Point, Polygon, box
from shapely.ops import unary_union
import shapely
from sexp import parse, find, find1
from board import load, CU

RHO = 1.72e-8          # Ом·м
PLATING = 0.025        # мм, металлизация отверстий
BOARD_T = 1.6          # мм


def zones(R, tracks, cfg):
    """тексты зон KiCad для сильноточных цепей"""
    pw = cfg['power']; out = []; info = []
    bx0, by0, bx1, by1 = R.rules.bbox
    inner = box(bx0, by0, bx1, by1).buffer(-(R.rules.v['edge'] + 0.05), join_style=2)
    for net in sorted(R.netid):
        I = R.rules.current(net)
        if I < pw['polygon_current']: continue
        for l in (0, 3):
            segs = [t for t in tracks if t[0] == net and t[1] == l]
            if not segs: continue
            g = unary_union([LineString([(t[2], t[3]), (t[4], t[5])]).buffer(t[6] / 2 + pw['polygon_margin'],
                                                                              cap_style=3, join_style=2) for t in segs])
            g = g.intersection(inner).simplify(0.05)
            polys = [g] if g.geom_type == 'Polygon' else list(getattr(g, 'geoms', []))
            for p in polys:
                if p.area < 1.0: continue
                pts = ' '.join('(xy %.4f %.4f)' % (x, y) for x, y in list(p.exterior.coords)[:-1])
                out.append('\t(zone\n\t\t(net "%s")\n\t\t(layer "%s")\n\t\t(uuid "%s")\n\t\t(name "PWR_%s")\n'
                           '\t\t(priority 1)\n\t\t(hatch edge 0.5)\n\t\t(connect_pads yes\n\t\t\t(clearance %.3f)\n\t\t)\n'
                           '\t\t(min_thickness 0.2)\n\t\t(fill yes\n\t\t\t(thermal_gap 0.5)\n\t\t\t(thermal_bridge_width 0.5)\n'
                           '\t\t\t(island_removal_mode 0)\n\t\t)\n\t\t(polygon\n\t\t\t(pts\n\t\t\t\t%s\n\t\t\t)\n\t\t)\n\t)'
                           % (net, CU[l], uuid.uuid4(), net.replace('+', 'P'), R.rules.v['clearance'], pts))
                info.append((net, CU[l], round(p.area, 1)))
    return out, info


# ---------------------------------------------------------------- IR drop on the filled board
def _net_copper(root, pads, net):
    """{слой: геометрия меди цепи}, переходы [(x, y, d, drill)], штыревые площадки [(x, y, drill, слои)]"""
    geo = {l: [] for l in range(4)}; vias = []; tht = []
    for s in find(root, 'segment'):
        n = find1(s, 'net')
        if not n or n[1] != net: continue
        a = find1(s, 'start'); b = find1(s, 'end'); w = float(find1(s, 'width')[1]); lay = find1(s, 'layer')[1]
        if lay in CU:
            geo[CU.index(lay)].append(LineString([(float(a[1]), float(a[2])), (float(b[1]), float(b[2]))]).buffer(w / 2, 8))
    for v in find(root, 'via'):
        n = find1(v, 'net')
        if not n or n[1] != net: continue
        at = find1(v, 'at'); d = float(find1(v, 'size')[1]); dr = float(find1(v, 'drill')[1])
        x, y = float(at[1]), float(at[2]); vias.append((x, y, d, dr))
        for l in range(4): geo[l].append(Point(x, y).buffer(d / 2, 16))
    for z in find(root, 'zone'):
        n = find1(z, 'net')
        if not n or n[1] != net: continue
        for fp in find(z, 'filled_polygon'):
            lay = find1(fp, 'layer')[1]; pts = find1(fp, 'pts')
            xy = [(float(p[1]), float(p[2])) for p in find(pts, 'xy')]
            if lay in CU and len(xy) >= 3: geo[CU.index(lay)].append(Polygon(xy).buffer(0))
    from post import padpoly
    for p in pads:
        if p.net != net: continue
        g = padpoly(p)
        for lay in p.layers: geo[CU.index(lay)].append(g)
        if p.kind == 'thru_hole' and p.drill: tht.append((p.x, p.y, p.drill, [CU.index(x) for x in p.layers]))
    return {l: unary_union(g) for l, g in geo.items() if g}, vias, tht


def ir_drop(board_path, net, sources, loads, res=0.1):
    """sources: [pad], loads: [(pad, I)] → dict(max_mV, per_load {ref.num: mV}, cells)"""
    import scipy.sparse as sp, scipy.sparse.linalg as spla
    root = parse(open(board_path, encoding='utf-8').read())[0]
    _, pads = load(board_path)
    pmap = {(p.ref, p.num, round(p.x, 3), round(p.y, 3)): p for p in pads}
    geo, vias, tht = _net_copper(root, pads, net)
    if not geo: return None
    allg = unary_union(list(geo.values()))
    x0, y0, x1, y1 = allg.bounds
    xs = np.arange(x0, x1 + res, res); ys = np.arange(y0, y1 + res, res)
    X, Y = np.meshgrid(xs, ys); H, W = X.shape
    idx = -np.ones((4, H, W), np.int64); n = 0
    for l, g in geo.items():
        m = shapely.contains_xy(g, X, Y)
        k = int(m.sum()); idx[l][m] = np.arange(n, n + k); n += k
    if n == 0: return None
    rows, cols, vals = [], [], []
    def cond(a, b, gval):
        rows.extend([a, b, a, b]); cols.extend([a, b, b, a]); vals.extend([gval, gval, -gval, -gval])
    t_mm = {0: 0.035, 1: 0.0175, 2: 0.0175, 3: 0.035}
    for l in range(4):
        I = idx[l]
        if (I >= 0).sum() == 0: continue
        gsq = (t_mm[l] * 1e-3) / RHO                       # 1/Ом на квадрат
        for (a, b) in ((I[:, :-1], I[:, 1:]), (I[:-1, :], I[1:, :])):
            m = (a >= 0) & (b >= 0)
            aa, bb = a[m], b[m]
            rows.extend(np.concatenate([aa, bb, aa, bb])); cols.extend(np.concatenate([aa, bb, bb, aa]))
            vals.extend(np.concatenate([np.full(len(aa), gsq)] * 2 + [np.full(len(aa), -gsq)] * 2))
    def barrel(x, y, drill, layers):
        i = int(round((x - x0) / res)); j = int(round((y - y0) / res))
        if not (0 <= i < W and 0 <= j < H): return
        nodes = [idx[l, j, i] for l in sorted(layers) if idx[l, j, i] >= 0]
        R_one = RHO * (BOARD_T / 3 * 1e-3) / (math.pi * drill * PLATING * 1e-6)    # между соседними слоями
        for a, b in zip(nodes, nodes[1:]): cond(a, b, 1.0 / R_one)
    for (x, y, d, dr) in vias: barrel(x, y, dr, range(4))
    for (x, y, dr, layers) in tht: barrel(x, y, dr, layers)
    G = sp.csr_matrix((np.array(vals, float), (np.array(rows), np.array(cols))), shape=(n, n))
    def pad_nodes(p):
        g = Point(p.x, p.y)
        out = []
        for lay in p.layers:
            l = CU.index(lay); i = int(round((p.x - x0) / res)); j = int(round((p.y - y0) / res))
            for dj in range(-2, 3):
                for di in range(-2, 3):
                    jj, ii = j + dj, i + di
                    if 0 <= jj < H and 0 <= ii < W and idx[l, jj, ii] >= 0: out.append(idx[l, jj, ii])
        return out
    fixed = sorted({k for p in sources for k in pad_nodes(p)})
    if not fixed: return None
    inj = np.zeros(n)
    for p, I in loads:
        ks = pad_nodes(p)
        if ks: inj[ks] -= I / len(ks)
    free = np.setdiff1d(np.arange(n), fixed)
    V = np.zeros(n)
    A = G[free][:, free].tocsc(); b = inj[free]
    V[free] = spla.spsolve(A, b)
    per = {}
    for p, I in loads:
        ks = pad_nodes(p)
        if ks: per['%s.%s' % (p.ref, p.num)] = round(-float(V[ks].mean()) * 1e3, 1)
    return dict(max_mV=round(max(per.values(), default=0.0), 1), per_load=per, cells=n)


def drop_report(board_path, R, cfg):
    """падение напряжения по меди для цепей с заданным источником"""
    pw = cfg['power']; rep = {}
    for net in sorted(R.netid):
        I = R.rules.current(net)
        src = cfg.lookup(pw['sources'], net)
        if I <= 0 or not src: continue
        srcs = [src] if isinstance(src, str) else list(src)
        pads = [p for p in R.pads if p.net == net]
        sources = [p for p in pads if p.ref in srcs]
        loadcfg = cfg.lookup(pw['loads'], net)
        if loadcfg:
            loads = []
            for ref, Ir in loadcfg.items():
                ps = [p for p in pads if p.ref == ref]
                loads += [(p, Ir / len(ps)) for p in ps]
        else:
            cand = [p for p in pads if p.ref not in srcs and p.ref[:1] in 'UJD']
            loads = [(p, I / len(cand)) for p in cand] if cand else []
        if not sources or not loads: continue
        r = ir_drop(board_path, net, sources, loads)
        if r:
            V = R.rules.nominal(net)
            r.update(I=I, V=V, pct=round(100 * r['max_mV'] / 1000 / V, 2) if V else None)
            rep[net] = r
    return rep
