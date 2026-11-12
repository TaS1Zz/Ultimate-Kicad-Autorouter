"""Независимая проверка без KiCad: зазоры между медью разных цепей, до края и отверстий, связность каждой цепи.
Запуск: python check.py out/demo.pkl example/demo_board.kicad_pcb [boards/demo.toml]"""
import sys, math, pickle, collections
from shapely.geometry import Point, LineString, box
from shapely.strtree import STRtree
from board import load, CU


def padgeom(p):
    w, h = (p.w, p.h) if round(p.ang) % 180 == 0 else (p.h, p.w)
    if p.shape == 'circle': return Point(p.x, p.y).buffer(p.w / 2, 32)
    if p.shape == 'oval':
        r = min(w, h) / 2; l = (max(w, h) - min(w, h)) / 2
        if l <= 0: return Point(p.x, p.y).buffer(r, 32)
        return (LineString([(p.x - l, p.y), (p.x + l, p.y)]) if w >= h else LineString([(p.x, p.y - l), (p.x, p.y + l)])).buffer(r, 32)
    return box(p.x - w / 2, p.y - h / 2, p.x + w / 2, p.y + h / 2)


def check(tracks, vias, pads, rules):
    CLR, EDGE, VIA_R = rules.v['clearance'], rules.v['edge'], rules.v['via_d'] / 2
    pairgap = {(i['p'], i['n']): i['s'] for i in rules.pair_info}
    pairgap.update({(b, a): s for (a, b), s in list(pairgap.items())})
    items = {l: [] for l in range(4)}; holes = []; uniq = 0
    for p in pads:
        if p.kind == 'np_thru_hole':
            holes.append((Point(p.x, p.y).buffer((p.drill or p.w) / 2, 32), p.ref)); continue
        g = padgeom(p); n = p.net
        if not n: uniq += 1; n = '__nc%d' % uniq
        for l in p.layers: items[CU.index(l)].append((g, n, 'pad %s.%s' % (p.ref, p.num)))
    for t in tracks:
        n, l, a, b, c, d, w = t[:7]
        g = Point(a, b).buffer(w / 2, 16) if math.hypot(c - a, d - b) < 1e-9 else LineString([(a, b), (c, d)]).buffer(w / 2, 16)
        items[l].append((g, n, 'trk'))
    for v in vias:
        g = Point(v[1], v[2]).buffer(VIA_R, 32)
        for l in range(4): items[l].append((g, v[0], 'via'))
    viol = []
    for l in range(4):
        geoms = [i[0] for i in items[l]]
        if not geoms: continue
        tree = STRtree(geoms)
        for k, (g, n, kind) in enumerate(items[l]):
            for j in tree.query(g.buffer(CLR)):
                if j <= k: continue
                g2, n2, k2 = items[l][j]
                if n2 == n or (kind.startswith('pad') and k2.startswith('pad')): continue
                lim = pairgap.get((n, n2), CLR)
                d = g.distance(g2)
                if d < lim - 1e-3: viol.append((CU[l], round(d, 3), n, kind, n2, k2))
    for (h, ref) in holes:
        for l in range(4):
            for (g, n, kind) in items[l]:
                if kind in ('trk', 'via'):
                    d = h.distance(g); lim = 0.25
                    if ref.startswith('MH'): d = Point(h.centroid).distance(g); lim = 2.2
                    if d < lim - 1e-3: viol.append((CU[l], round(d, 3), 'HOLE', ref, n, kind))
    edge = box(*rules.bbox).exterior
    for l in range(4):
        for (g, n, kind) in items[l]:
            if kind in ('trk', 'via') and g.distance(edge) < EDGE - 1e-3:
                viol.append((CU[l], round(g.distance(edge), 3), 'EDGE', kind, n, ''))
    # связность; GND соединена плоскостью: всё, что на слое плоскости (переходы, штыревые площадки), — одно целое
    nets = collections.defaultdict(list)
    for l in range(4):
        for (g, n, kind) in items[l]: nets[n].append((l, g, kind))
    bad = []
    for n, its in nets.items():
        if n.startswith('__nc'): continue
        N = len(its); par = list(range(N))
        def f(a):
            while par[a] != a: par[a] = par[par[a]]; a = par[a]
            return a
        byl = collections.defaultdict(list)
        for i in range(N): byl[its[i][0]].append(i)
        for l, idx in byl.items():
            gs = [its[i][1] for i in idx]; tr = STRtree(gs)
            for i in idx:
                for b in tr.query(its[i][1]):
                    if its[i][1].distance(gs[b]) < 1e-6: par[f(i)] = f(idx[b])
        if n == 'GND':
            idx = [i for i in range(N) if its[i][0] in rules.planes]
            for i in idx[1:]: par[f(i)] = f(idx[0])
        byid = collections.defaultdict(list)
        for i in range(N): byid[id(its[i][1])].append(i)
        for v in byid.values():
            for i in v[1:]: par[f(i)] = f(v[0])
        comps = collections.defaultdict(list)
        for i in range(N):
            if its[i][2].startswith('pad'): comps[f(i)].append(its[i][2])
        if len(comps) > 1: bad.append((n, len(comps), [sorted(set(g))[:3] for g in comps.values()][:4]))
    return viol, bad


if __name__ == '__main__':
    import config
    from rules import Rules
    tracks, vias = pickle.load(open(sys.argv[1], 'rb'))[:2]
    cfg = config.load(sys.argv[3]) if len(sys.argv) > 3 else None
    _, pads = load(sys.argv[2])
    viol, bad = check(tracks, vias, pads, Rules(sys.argv[2], cfg))
    print('violations', len(viol))
    for v in viol[:40]: print(v)
    print('unconnected nets', len(bad))
    for b in bad: print(b)
