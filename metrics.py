"""Метрики качества трассировки (без KiCad).
Запуск: python metrics.py out/demo.pkl example/demo_board.kicad_pcb
  off_angle — отрезки не под 0/45/90° (кривые подходы к площадкам);
  jogs      — изломы на отрезках короче 0,5 мм: «ступеньки»;
  bends     — изломы всего;
  detour    — длина трасс цепи / длина октилинейного MST по её площадкам (среднее и худшие);
  bus       — для групп шин доля длины, идущей параллельно соседям на одном слое в пределах 1,5 мм."""
import sys, math, pickle, collections


def seg_angle_ok(t):
    dx, dy = t[4] - t[2], t[5] - t[3]
    if math.hypot(dx, dy) < 1e-6: return True
    a = math.degrees(math.atan2(dy, dx)) % 45
    return min(a, 45 - a) < 0.5


def octile(a, b):
    dx, dy = abs(a[0] - b[0]), abs(a[1] - b[1])
    return max(dx, dy) + (math.sqrt(2) - 1) * min(dx, dy)


def mst_len(pts):
    if len(pts) < 2: return 0.0
    inT = [False] * len(pts); d = [1e18] * len(pts); d[0] = 0; tot = 0
    for _ in pts:
        k = min((i for i in range(len(pts)) if not inT[i]), key=lambda i: d[i])
        inT[k] = True; tot += d[k]
        for i in range(len(pts)):
            if not inT[i]: d[i] = min(d[i], octile(pts[k], pts[i]))
    return tot


def compute(tracks, vias, pads, groups=None):
    from rules import bus_groups
    bynet = collections.defaultdict(list)
    for t in tracks: bynet[t[0]].append(t)
    off = sum(1 for t in tracks if not seg_angle_ok(t))
    jogs = bends = 0; detours = []
    padpts = collections.defaultdict(list)
    for p in pads:
        if p.net and p.kind != 'np_thru_hole': padpts[p.net].append((p.x, p.y))
    for n, ts in bynet.items():
        for l in set(t[1] for t in ts):
            adj = collections.defaultdict(list)
            for t in ts:
                if t[1] != l: continue
                a = (round(t[2], 3), round(t[3], 3)); b = (round(t[4], 3), round(t[5], 3))
                adj[a].append(b); adj[b].append(a)
            for v, nb in adj.items():
                if len(nb) != 2: continue
                a, b = nb
                d1 = (v[0] - a[0], v[1] - a[1]); d2 = (b[0] - v[0], b[1] - v[1])
                if abs(d1[0] * d2[1] - d1[1] * d2[0]) > 1e-6:
                    bends += 1
                    if min(math.hypot(*d1), math.hypot(*d2)) < 0.5: jogs += 1
        L = sum(math.hypot(t[4] - t[2], t[5] - t[3]) for t in ts)
        m = mst_len(padpts[n])
        if m > 5 and n != 'GND': detours.append((L / m, n))
    detours.sort(reverse=True)
    from shapely.geometry import LineString
    from shapely.ops import unary_union
    bus = {}
    for g, members in (groups or bus_groups(bynet.keys())).items():
        tot = par = 0.0; geo = {}
        for n in members:
            for t in bynet.get(n, []):
                geo.setdefault((n, t[1]), []).append(LineString([(t[2], t[3]), (t[4], t[5])]))
        for (n, l), ls in geo.items():
            others = [s for (m, l2), v in geo.items() if m != n and l2 == l for s in v]
            if not others: tot += sum(s.length for s in ls); continue
            near = unary_union(others).buffer(1.5)
            for s in ls:
                tot += s.length
                if s.length > 0.3: par += s.intersection(near).length
        if tot > 0: bus[g] = round(100 * par / tot)
    return dict(segments=len(tracks), vias=len(vias), off_angle=off, bends=bends, jogs=jogs,
                detour_mean=round(sum(d for d, _ in detours) / max(1, len(detours)), 3),
                detour_worst=[(n, round(d, 2)) for d, n in detours[:5]], bus_parallel=bus,
                length_mm=round(sum(math.hypot(t[4] - t[2], t[5] - t[3]) for t in tracks), 1))


def main(pkl, pcb):
    from board import load
    tracks, vias = pickle.load(open(pkl, 'rb'))[:2]
    _, pads = load(pcb)
    m = compute(tracks, vias, pads)
    print('отрезков %d  переходов %d  длина %.0f мм' % (m['segments'], m['vias'], m['length_mm']))
    print('off-angle (не 0/45/90°): %d' % m['off_angle'])
    print('изломов: %d   ступенек (<0,5 мм): %d' % (m['bends'], m['jogs']))
    print('detour средний %.2f, худшие: %s' % (m['detour_mean'], ', '.join('%s %.2f' % x for x in m['detour_worst'])))
    for g, v in m['bus_parallel'].items(): print('шина %-9s параллельно %3d%%' % (g, v))


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2])
