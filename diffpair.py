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
        lp = path_length(tracks, vias, P, refs[a][P][0], refs[b][P][0])
        ln = path_length(tracks, vias, N, refs[a][N][0], refs[b][N][0])
        d = dict(pair=(P, N), ends=(a, b), w=info['w'], s=info['s'], zdiff=round(info['z'], 1), target=info['target'],
                 len_p=None if lp is None else round(lp, 2), len_n=None if ln is None else round(ln, 2))
        if lp is not None and ln is not None:
            d['skew_mm'] = round(abs(lp - ln), 3); d['skew_ps'] = round(abs(lp - ln) * info['ps_mm'], 1)
            d['ok'] = d['skew_mm'] <= R.cfg['nets']['skew_max']
        out.append(d)
    return out
