import pytest
import rules, impedance, config


def test_ipc2221_outer_1oz_1A():
    # IPC-2221, ΔT = 10 °C, 35 мкм: 1 А ≈ 0.30 мм
    assert rules.ipc_width(1.0, 0.035, inner=False) == pytest.approx(0.30, abs=0.01)


def test_ipc2221_inner_needs_wider_copper():
    assert rules.ipc_width(1.0, 0.035, inner=True) > 1.9 * rules.ipc_width(1.0, 0.035, inner=False)


def test_drop_width():
    # 1 А на 100 мм при допуске 0.1 В и 35 мкм: w = ρLI/(tV) ≈ 0.49 мм
    assert rules.drop_width(1.0, 100, 0.035, 0.1) == pytest.approx(0.491, abs=0.005)


def test_ceil_to():
    assert rules.ceil_to(0.301) == 0.35 and rules.ceil_to(0.30) == 0.3


def test_zdiff_reference_value():
    # w = 0.25, s = 0.15 мм над препрегом 0.2104 мм, εr 4.4 → ≈ 92 Ом (IPC-2141)
    assert impedance.zdiff(0.25, 0.15, 0.2104, 4.4) == pytest.approx(91.7, abs=1.0)


def test_design_pair_hits_target():
    w, s, z = impedance.design_pair(90, 0.2104, 4.4, 0.035, w_min=0.1, s_min=0.2)
    assert s >= 0.2 and abs(z - 90) < 1.0


def test_demo_rules_from_board_and_config():
    cfg = config.load('boards/demo.toml')
    R = rules.Rules(cfg.pcb(), cfg)
    assert R.bbox == (100.0, 100.0, 200.0, 170.0)
    assert R.planes == {1}                          # зона GND на In1 — плоскость
    assert R.fast_layers() == {0}                   # у In2 ближе сигнальный B, чем плоскость
    assert R.current('VBUS') == pytest.approx(1.31)
    st = R.widths('USB_D+', 50)[0]
    assert st[3] is None and st[2] is None and st[0]
    st = R.widths('VBUS', 80)[0]
    # 1.31 А: снаружи по нагреву ≈ 0.45 мм; внутри (0.5 oz) нужно шире 1 мм — слой запрещён
    assert st[2] is None and st[0] == pytest.approx(rules.ceil_to(rules.ipc_width(1.31, 0.035, False)), abs=0.051)
