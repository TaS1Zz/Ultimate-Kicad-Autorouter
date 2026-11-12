"""Регрессия на тестовой плате: полная трассировка и пороги качества."""
import os, shutil, pytest
import config, metrics, check, diffpair, pipeline
from board import load


def test_config_autodetect():
    cfg = config.load('boards/demo.toml')
    _, pads = load(cfg.pcb())
    plan = config.fanout_plan(cfg, pads, power_nets={'VBUS', '+3V3'})
    assert set(plan) == {'U1', 'U2', 'J1'}
    assert all(n in ('A5', 'A6', 'A7', 'B5', 'B6', 'B7') for n in plan['J1'])
    assert len(config.thermal_vias(cfg, pads, 0.6)) == 4      # 2×2 под площадкой QFN-32


@pytest.fixture(scope='module')
def routed(tmp_path_factory):
    import route_board
    out = str(tmp_path_factory.mktemp('demo') / 'demo')
    cfg = config.load('boards/demo.toml')
    R, state = route_board.route(cfg, out, threads=2)
    return R, cfg, out


@pytest.mark.slow
def test_demo_routes_completely(routed):
    R, cfg, out = routed
    assert R.fail == []
    tr, vi = R.export()
    viol, bad = check.check(tr, vi, R.pads, R.rules)
    assert viol == [] and bad == []


@pytest.mark.slow
def test_demo_quality(routed):
    R, cfg, out = routed
    tr, vi = R.export()
    m = metrics.compute(tr, vi, R.pads, R.groups)
    assert m['off_angle'] == 0
    assert m['detour_mean'] < 1.2
    assert m['vias'] < 120
    assert m['bus_parallel'].get('LED_K', 0) >= 40


@pytest.mark.slow
def test_demo_pair_skew(routed):
    R, cfg, out = routed
    tr, vi = R.export()
    rep = diffpair.report(R, tr, vi)[0]
    assert rep['skew_mm'] <= cfg['nets']['skew_max'] + 0.05


@pytest.mark.slow
@pytest.mark.skipif(pipeline.kicad_cli() is None, reason='нет kicad-cli')
def test_demo_kicad_drc(routed):
    R, cfg, out = routed
    rep = pipeline.finish_and_check(R, cfg, out, drc=True)
    by = rep['drc']['by_type']
    for t in ('clearance', 'unconnected_items', 'track_dangling', 'via_dangling', 'hole_to_hole', 'shorting_items'):
        assert by.get(t, 0) == 0, (t, by)
    assert rep['ir_drop']['VBUS']['pct'] < 2.0
