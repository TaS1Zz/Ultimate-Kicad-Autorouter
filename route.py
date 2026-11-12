"""Сетка и карты расстояний до меди (общая основа маршрутизатора)."""
import math
import numpy as np

RES = 0.1
X0, Y0, X1, Y1 = 0.0, 0.0, 100.0, 100.0   # сетка; Router2 перезаписывает из Edge.Cuts (configure)
BX0, BY0, BX1, BY1 = X0, Y0, X1, Y1         # настоящий контур платы
W = int(round((X1 - X0) / RES)) + 1
H = int(round((Y1 - Y0) / RES)) + 1
CLR = 0.2            # copper-copper clearance
EDGE = 0.5           # copper to board edge
MARG = 0.01          # numeric margin for grid checks
CAP = 1.2            # distance cap stored in maps
VIA_D, VIA_DRILL = 0.6, 0.3

xs = X0 + np.arange(W) * RES
ys = Y0 + np.arange(H) * RES


def gx(x): return int(round((x - X0) / RES))
def gy(y): return int(round((y - Y0) / RES))


def configure(rules, pads=()):
    """Подстроить сетку и правила под плату: контур из Edge.Cuts, зазоры/переходы из Rules.
    Начало сетки сдвигается (меньше чем на шаг) так, чтобы центры узких выводов попадали точно в узлы:
    иначе ближайший к центру узел смещён к соседнему выводу и трасса не выходит из площадки по зазору.
    Расстояние до края считается от настоящего контура (BX0..BY1), а не от сетки.
    Вызывается из Router2 до построения карт; модули, которым нужны текущие значения,
    обращаются к ним как route.X0 и т. д."""
    global X0, Y0, X1, Y1, W, H, xs, ys, CLR, EDGE, VIA_D, VIA_DRILL, BX0, BY0, BX1, BY1
    BX0, BY0, BX1, BY1 = rules.bbox
    def phase(vals, base):
        from collections import Counter
        c = Counter(round(((v - base) % RES) / 0.005) * 0.005 % RES for v in vals)
        return c.most_common(1)[0][0] if c else 0.0
    fine = [p for p in pads if p.layers and min(p.w, p.h) <= 0.4]
    dx, dy = phase([p.x for p in fine], BX0), phase([p.y for p in fine], BY0)
    X0 = BX0 + dx - (RES if dx > 1e-9 else 0.0); Y0 = BY0 + dy - (RES if dy > 1e-9 else 0.0)
    X1, Y1 = BX1, BY1
    W = int(math.ceil((X1 - X0) / RES - 1e-9)) + 1
    H = int(math.ceil((Y1 - Y0) / RES - 1e-9)) + 1
    xs = X0 + np.arange(W) * RES
    ys = Y0 + np.arange(H) * RES
    CLR, EDGE = rules.v['clearance'], rules.v['edge']
    VIA_D, VIA_DRILL = rules.v['via_d'], rules.v['via_drill']


class Maps:
    def __init__(self):
        self.d1 = np.full((4, H, W), CAP, np.float32)
        self.d2 = np.full((4, H, W), CAP, np.float32)
        self.n1 = np.zeros((4, H, W), np.int32)
        # board edge distance (same for every layer)
        ex = np.minimum(xs - BX0, BX1 - xs)[None, :]
        ey = np.minimum(ys - BY0, BY1 - ys)[:, None]
        self.edge = np.minimum(ex, ey).astype(np.float32)

    def window(self, x0, y0, x1, y1):
        i0 = max(0, int(math.floor((x0 - X0) / RES))); i1 = min(W, int(math.ceil((x1 - X0) / RES)) + 1)
        j0 = max(0, int(math.floor((y0 - Y0) / RES))); j1 = min(H, int(math.ceil((y1 - Y0) / RES)) + 1)
        return i0, i1, j0, j1

    def add(self, layer, net, bbox, distfn):
        R = CAP
        i0, i1, j0, j1 = self.window(bbox[0] - R, bbox[1] - R, bbox[2] + R, bbox[3] + R)
        if i0 >= i1 or j0 >= j1:
            return
        X, Y = np.meshgrid(xs[i0:i1], ys[j0:j1])
        d = np.minimum(distfn(X, Y), CAP).astype(np.float32)
        d1 = self.d1[layer, j0:j1, i0:i1]; d2 = self.d2[layer, j0:j1, i0:i1]; n1 = self.n1[layer, j0:j1, i0:i1]
        same = n1 == net
        closer = (~same) & (d < d1)
        other = (~same) & (~closer)
        nd1 = np.where(same, np.minimum(d1, d), np.where(closer, d, d1))
        nd2 = np.where(closer, d1, np.where(other, np.minimum(d2, d), d2))
        nn1 = np.where(closer, net, n1)
        d1[...] = nd1; d2[...] = nd2; n1[...] = nn1

    def dn(self, layer, net, sl=(slice(None), slice(None))):
        return np.where(self.n1[layer][sl] == net, self.d2[layer][sl], self.d1[layer][sl])


def rect_dist(cx, cy, w, h, ang):
    a = math.radians(ang); c, s = abs(math.cos(a)), abs(math.sin(a))
    if c > 0.5: hw, hh = w / 2, h / 2
    else: hw, hh = h / 2, w / 2
    def f(X, Y):
        dx = np.maximum(np.abs(X - cx) - hw, 0); dy = np.maximum(np.abs(Y - cy) - hh, 0)
        inside = (np.abs(X - cx) <= hw) & (np.abs(Y - cy) <= hh)
        dd = np.hypot(dx, dy)
        dneg = -np.minimum(hw - np.abs(X - cx), hh - np.abs(Y - cy))
        return np.where(inside, dneg, dd)
    return f, (cx - hw, cy - hh, cx + hw, cy + hh)


def circ_dist(cx, cy, r):
    return (lambda X, Y: np.hypot(X - cx, Y - cy) - r), (cx - r, cy - r, cx + r, cy + r)


def oval_dist(cx, cy, w, h, ang):
    a = math.radians(ang); c = abs(math.cos(a))
    if c < 0.5: w, h = h, w
    if w >= h:
        r = h / 2; l = (w - h) / 2; ax, ay = cx - l, cy; bx, by = cx + l, cy
    else:
        r = w / 2; l = (h - w) / 2; ax, ay = cx, cy - l; bx, by = cx, cy + l
    f, _ = seg_dist(ax, ay, bx, by, 2 * r)
    return f, (cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2)


def seg_dist(ax, ay, bx, by, width):
    vx, vy = bx - ax, by - ay; L2 = vx * vx + vy * vy
    def f(X, Y):
        if L2 == 0:
            return np.hypot(X - ax, Y - ay) - width / 2
        t = np.clip(((X - ax) * vx + (Y - ay) * vy) / L2, 0, 1)
        return np.hypot(X - ax - t * vx, Y - ay - t * vy) - width / 2
    return f, (min(ax, bx) - width / 2, min(ay, by) - width / 2, max(ax, bx) + width / 2, max(ay, by) + width / 2)


def pad_shape(p):
    if p.shape in ('rect', 'roundrect', 'custom', 'trapezoid'):
        return rect_dist(p.x, p.y, p.w, p.h, p.ang)
    if p.shape == 'circle':
        return circ_dist(p.x, p.y, p.w / 2)
    if p.shape == 'oval':
        return oval_dist(p.x, p.y, p.w, p.h, p.ang)
    raise ValueError(p.shape)
