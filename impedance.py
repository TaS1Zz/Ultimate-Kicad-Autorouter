"""Импеданс микрополоска и дифпары по стеку (IPC-2141 / Wadell, точность ±5–10 %).

Z0 (микрополосок)      = 87 / sqrt(εr + 1.41) · ln(5.98·h / (0.8·w + t)),  0.1 < w/h < 2
Zdiff (связанная пара) = 2·Z0 · (1 − 0.48·exp(−0.96·s/h))
Внутренний слой рядом с плоскостью считается как микрополосок над ней (вторая сторона — не плоскость):
это оценка, для серийного изделия импеданс надо заказать у производителя по его расчёту стека.
Скорость сигнала v = c / sqrt(εeff), εeff ≈ 0.475·εr + 0.67 (микрополосок на FR-4).
"""
import math

C_MM_PS = 0.299792458            # мм за пикосекунду


def z0_microstrip(w, h, er, t=0.035):
    return 87.0 / math.sqrt(er + 1.41) * math.log(5.98 * h / (0.8 * w + t))


def zdiff(w, s, h, er, t=0.035):
    return 2 * z0_microstrip(w, h, er, t) * (1 - 0.48 * math.exp(-0.96 * s / h))


def eps_eff(er):
    return 0.475 * er + 0.67


def ps_per_mm(er):
    return math.sqrt(eps_eff(er)) / C_MM_PS


def design_pair(target, h, er, t, w_min, s_min, w_max=0.6, s_max=0.6, step=0.005):
    """подобрать (w, s) для Zdiff = target: минимальная ошибка, при равной — узкий зазор (плотная связь)"""
    best = None
    w = w_min
    while w <= w_max + 1e-9:
        s = s_min
        while s <= s_max + 1e-9:
            z = zdiff(w, s, h, er, t)
            key = (round(abs(z - target), 1), s, -w)
            if best is None or key < best[0]: best = (key, round(w, 3), round(s, 3), z)
            s += step
        w += step
    return best[1], best[2], best[3]
