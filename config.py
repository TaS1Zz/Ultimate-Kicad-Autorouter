"""Настройки платы: TOML-файл рядом с платой (boards/*.toml) поверх значений по умолчанию.

Всё, что раньше было зашито в сценарий трассировки, описывается здесь, а то, что можно найти на плате
само, находится автоматически:
  * микросхемы для fanout — футпринты U* с SMD-выводами уже 0.35 мм;
  * переходы под открытой площадкой — сетка внутри GND-площадки микросхемы площадью больше 4 мм²;
  * плоскости — внутренние слои, на которых есть зона цепи GND и нет других зон.
"""
import fnmatch, math, os, re
try:
    import tomllib
except ModuleNotFoundError:          # Python < 3.11
    import tomli as tomllib

DEFAULTS = {
    'board': {'file': ''},
    'rules': {},                     # clearance, edge, track, track_min, via_d, via_drill — поверх .kicad_pro
    'stackup': {                     # JLC04161H-7628 (JLCPCB, 4 слоя, 1.6 мм) — если в плате нет (stackup)
        'copper': {'F.Cu': 0.035, 'In1.Cu': 0.0175, 'In2.Cu': 0.0175, 'B.Cu': 0.035},
        'dielectric': [              # между соседними слоями меди, сверху вниз: толщина, εr
            {'between': ['F.Cu', 'In1.Cu'], 'h': 0.2104, 'er': 4.4},
            {'between': ['In1.Cu', 'In2.Cu'], 'h': 1.065, 'er': 4.6},
            {'between': ['In2.Cu', 'B.Cu'], 'h': 0.2104, 'er': 4.4},
        ],
        'planes': [],                # пусто — найти автоматически
    },
    'power': {
        'currents': {},              # {"VBUS": 1.5}; шаблоны fnmatch, поверх rules.CURRENTS
        'nominal': {},               # {"VBUS": 5.0}
        'sources': {},               # {"VBUS": "J1"} — откуда втекает ток (для расчёта падения)
        'loads': {},                 # {"VBUS": {"U2": 0.2}} — потребители; по умолчанию U*/J*/D* поровну
        'polygon_current': 1.0,      # от этого тока цепь получает ещё и полигон на внешнем слое
        'polygon_margin': 0.8,       # на сколько полигон шире трассы с каждой стороны, мм
    },
    'fanout': {
        'auto': True,
        'mode': 'power',             # power — переходы только у выводов GND и питания (им нужна плоскость);
                                     # all — у всех выводов (плотные BGA/QFN, когда сигналам не выйти по своему слою)
        'refs': [],                  # дополнительно (разъёмы): ["J1"]
        'pins': {},                  # ограничить выводы: {"J1": ["A5", "A6"]}
        'exclude': {},               # не выводить: {"U1": ["7", "8"]}
    },
    'thermal': {'auto': True, 'pitch': 1.5, 'refs': []},
    'nets': {
        'fast': [],                  # быстрые фронты: только слои рядом с плоскостью
        'diff_pairs': [],            # [["USB_D+", "USB_D-", 90.0]] — пара и целевой Zdiff, Ом
        'skew_max': 0.15,            # допустимая разница длин пары, мм
        'pair_gap_min': 0,           # мин. зазор внутри пары, мм; 0 — общий зазор платы (без отдельного правила DRC)
        'order_first': [],           # эти цепи — сразу после питания
    },
    'route': {
        'negotiate_iters': 30,       # PathFinder: максимум проходов
        'threads': 0,                # 0 — по числу ядер
    },
    'drc': {'loop': True, 'iters': 3},
}


def _merge(a, b):
    out = dict(a)
    for k, v in b.items():
        out[k] = _merge(a[k], v) if isinstance(v, dict) and isinstance(a.get(k), dict) else v
    return out


class Config(dict):
    def __init__(self, data, path=None):
        super().__init__(data); self.path = path

    def pcb(self):
        f = self['board']['file']
        if self.path and f and not os.path.isabs(f):
            f = os.path.normpath(os.path.join(os.path.dirname(self.path), f))
        return f

    # ---------- matching helpers
    @staticmethod
    def match(name, patterns):
        return any(fnmatch.fnmatchcase(name, p) for p in patterns)

    def lookup(self, table, net, default=None):
        """значение из таблицы {шаблон: значение} для цепи; точное имя важнее шаблона"""
        if net in table: return table[net]
        for p, v in table.items():
            if fnmatch.fnmatchcase(net, p): return v
        return default


def load(path=None, overrides=None):
    data = DEFAULTS
    if path:
        with open(path, 'rb') as f: data = _merge(DEFAULTS, tomllib.load(f))
    if overrides: data = _merge(data, overrides)
    return Config(data, path)


# ---------------------------------------------------------------- auto-detection on the board
def fanout_plan(cfg, pads, power_nets=()):
    """{ref: [номера выводов] или None (все)} — какие выводы выводить на переходы"""
    fo = cfg['fanout']; plan = {}
    if fo['auto']:
        byref = {}
        for p in pads: byref.setdefault(p.ref, []).append(p)
        for ref, ps in byref.items():
            if not ref.startswith('U'): continue
            fine = [p for p in ps if p.kind == 'smd' and p.net and min(p.w, p.h) <= 0.35]
            if len(fine) < 4: continue
            if fo['mode'] == 'all': plan[ref] = None
            else: plan[ref] = [p.num for p in fine if p.net == 'GND' or (power_nets and p.net in power_nets)]
    for ref in fo['refs']: plan.setdefault(ref, None)
    for ref, pins in fo['pins'].items(): plan[ref] = list(pins)
    for ref, ex in fo['exclude'].items():
        if ref in plan:
            allp = plan[ref] or [p.num for p in pads if p.ref == ref and p.net]
            plan[ref] = [n for n in allp if n not in ex]
    return plan


def thermal_vias(cfg, pads, via_d):
    """координаты переходов под открытыми GND-площадками микросхем"""
    th = cfg['thermal']; out = []
    if not th['auto'] and not th['refs']: return out
    for p in pads:
        if p.net != 'GND' or p.kind != 'smd' or not p.ref.startswith('U'): continue
        if th['refs'] and p.ref not in th['refs']: continue
        w, h = (p.w, p.h) if round(p.ang) % 180 == 0 else (p.h, p.w)
        if w * h < 4.0: continue
        pitch = th['pitch']
        nx = max(1, int((w - via_d) // pitch) + 1); ny = max(1, int((h - via_d) // pitch) + 1)
        for a in range(nx):
            for b in range(ny):
                out.append((p.x + (a - (nx - 1) / 2) * pitch, p.y + (b - (ny - 1) / 2) * pitch))
    return out
