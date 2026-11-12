"""Ядро A* напрямую: маленькие сетки, 1–2 слоя."""
import numpy as np
import pytest
import route2
from route2 import _lib, P_u8, P_i, P_f


def run(allow, viaok, src, tgt, cost=None, b45=0.0, b90=0.0, viac=5.0, dircost=None):
    L, H, W = allow.shape
    T = np.zeros_like(allow); T[tgt] = 1
    S = np.array([c for s in src for c in s], np.int32)
    k = np.nonzero(T)
    dc = np.array(dircost or [1.0, 1.0, 1.0] * L, np.float32)
    out = np.zeros(3 * 10000, np.int32)
    allow = np.ascontiguousarray(allow.astype(np.uint8)); T = np.ascontiguousarray(T.astype(np.uint8))
    V = np.ascontiguousarray(viaok.astype(np.uint8))
    c = None if cost is None else np.ascontiguousarray(cost.astype(np.float32))
    n = _lib.astar(W, H, L, allow.ctypes.data_as(P_u8), V.ctypes.data_as(P_u8), T.ctypes.data_as(P_u8),
                   len(src), S.ctypes.data_as(P_i), dc.ctypes.data_as(P_f), b45, b90, viac,
                   int(k[2].min()), int(k[2].max()), int(k[1].min()), int(k[1].max()), 10 ** 7,
                   out.ctypes.data_as(P_i), 10000, c.ctypes.data if c is not None else None, 1.0, None)
    return n, out[:3 * max(n, 0)].reshape(-1, 3)


def test_straight_line():
    a = np.ones((1, 5, 20), bool)
    n, p = run(a, np.zeros((5, 20), bool), [(0, 2, 0)], (0, 2, 19))
    assert n == 20 and set(p[:, 1]) == {2}


def test_detour_around_wall():
    a = np.ones((1, 9, 9), bool); a[0, 0:8, 4] = False           # стена с проходом внизу
    n, p = run(a, np.zeros((9, 9), bool), [(0, 0, 0)], (0, 0, 8))
    assert n > 0 and all(a[0, j, i] for _, j, i in p) and max(p[:, 1]) == 8


def test_unreachable_returns_minus_one():
    a = np.ones((1, 5, 9), bool); a[0, :, 4] = False
    n, _ = run(a, np.zeros((5, 9), bool), [(0, 2, 0)], (0, 2, 8))
    assert n == -1


def test_via_switches_layer():
    a = np.ones((2, 5, 9), bool); a[0, :, 4] = False               # на верхнем стена, нижний свободен
    v = np.zeros((5, 9), bool); v[2, 2] = v[2, 6] = True
    n, p = run(a, v, [(0, 2, 0)], (0, 2, 8))
    assert n > 0 and 1 in set(p[:, 0])


def test_cost_map_steers_path():
    a = np.ones((1, 7, 15), bool)
    cost = np.zeros((1, 7, 15), np.float32); cost[0, 3, 1:14] = 50  # середина дорогая
    n, p = run(a, np.zeros((7, 15), bool), [(0, 3, 0)], (0, 3, 14), cost=cost)
    assert n > 0 and sum(1 for _, j, i in p if j == 3 and 0 < i < 14) == 0


def test_bend_penalty_prefers_single_bend():
    a = np.ones((1, 10, 10), bool)
    n, p = run(a, np.zeros((10, 10), bool), [(0, 0, 0)], (0, 9, 9), b45=5, b90=12, dircost=[1.0, 1.0, 1.0])
    d = [(int(np.sign(b[1] - a[1])), int(np.sign(b[2] - a[2]))) for a, b in zip(p, p[1:])]
    assert sum(1 for x, y in zip(d, d[1:]) if x != y) <= 1           # чистая диагональ, без зигзага
