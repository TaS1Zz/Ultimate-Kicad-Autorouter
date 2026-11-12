import math
from merge import merge, union_overlaps
from post import trim_dangling, postprocess


class Pad:
    def __init__(self, net, x, y, w=1.0, h=1.0, ref='R1', num='1', layers=('F.Cu',)):
        self.net, self.x, self.y, self.w, self.h = net, x, y, w, h
        self.ref, self.num, self.layers, self.kind, self.shape, self.ang = ref, num, list(layers), 'smd', 'rect', 0.0


def test_union_overlapping_collinear():
    S = union_overlaps([((0.0, 0.0), (5.0, 0.0)), ((3.0, 0.0), (8.0, 0.0))])
    assert S == [((0.0, 0.0), (8.0, 0.0))]


def test_touching_segments_kept_for_junction():
    S = union_overlaps([((0.0, 0.0), (5.0, 0.0)), ((5.0, 0.0), (8.0, 0.0))])
    assert len(S) == 2                              # касание — узел, его сливает merge(), а не union


def test_merge_collinear_chain():
    tr = [('N', 0, 0, 0, 1, 0, 0.25), ('N', 0, 1, 0, 3, 0, 0.25), ('N', 0, 3, 0, 3, 2, 0.25)]
    out = merge(tr, [])
    assert len(out) == 2 and any(abs(t[4] - t[2]) == 3 for t in out)


def test_trim_dangling_shortens_to_pad():
    pads = [Pad('N', 0, 0), Pad('N', 10, 0, ref='R2')]
    tr = [('N', 0, 0.0, 0.0, 10.0, 0.0, 0.25), ('N', 0, 10.0, 0.0, 12.0, 0.0, 0.25)]   # хвост за площадкой R2
    out, n = trim_dangling(tr, [], pads)
    assert n >= 1 and max(max(t[2], t[4]) for t in out) <= 10.0 + 1e-6


def test_postprocess_keeps_connectivity():
    pads = [Pad('N', 0, 0), Pad('N', 10, 0, ref='R2')]
    tr = [('N', 0, 0.0, 0.0, 10.0, 0.0, 0.25)]
    out, vi = postprocess(tr, [], pads)
    assert len(out) == 1
