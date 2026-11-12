"""Правила трассировки: геометрия платы, стек, зазоры, ширины по току и падению напряжения, быстрые цепи, дифпары.

Откуда берутся числа
--------------------
* Контур — bbox линий слоя Edge.Cuts; толщина меди слоёв — из (stackup) платы, иначе из конфига
  (по умолчанию JLC04161H-7628: 1 oz снаружи, 0.5 oz внутри).
* Зазоры, ширина по умолчанию, переходы — из .kicad_pro рядом с платой (net_settings / design_settings),
  затем из [rules] конфига, иначе значения по умолчанию ниже (под возможности JLCPCB, с запасом).
* Ширина питания — максимум из двух условий, отдельно для каждого слоя:
    1) нагрев: IPC-2221, I = k·ΔT^0.44·A^0.725 (A в mil², k = 0.048 наружный слой, 0.024 внутренний), ΔT = 10 °C;
    2) падение напряжения: R = ρ·L/(w·t), ρ(Cu) = 1.72e-8 Ом·м — 35 мкм меди дают ≈ 0.5 мОм на квадрат.
       L = 1.2 × полупериметр bbox площадок цепи; нагрузки распределены вдоль цепи, поэтому берётся I/2;
       допуск — DROP_FRAC от номинала. Ширина не больше WMAX: дальше нужен полигон (power.py).
  Если целевая ширина не проходит, ступени: ширина по нагреву → сигнальная (с предупреждением в отчёте).
* Быстрые цепи (конфиг nets.fast) идут только по слоям, соседним с плоскостью: у них должен быть
  опорный слой для обратного тока. В стеке F / GND / In2 / B слой B граничит с сигнальным In2 — для них запрещён.
* Дифпары: ширина и зазор подбираются под целевой Zdiff по стеку (impedance.py).
* Общие принципы разводки (их же излагают Хоровиц и Хилл в «Искусстве схемотехники»):
  сплошной слой земли и короткие широкие выводы на него от каждой GND-площадки — переход как можно ближе
  к площадке, в первую очередь у развязывающих конденсаторов; питание широкое и короткое; у быстрых сигналов —
  опорная плоскость рядом; медленные цифровые линии (матрица, I²C, светодиоды) — минимальной ширины.
"""
import math, json, os, re
from sexp import parse, find, find1
import impedance

CU_NAMES = ['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu']

# ---------------------------------------------------------------- defaults (мм)
DEFAULTS = dict(
    clearance=0.2,        # медь–медь
    edge=0.5,             # медь–край платы
    track=0.25,           # сигнальная трасса
    track_min=0.2,        # меньше не сужаем никогда (кроме выхода из площадки уже её самой)
    via_d=0.6, via_drill=0.3,
    gnd_stub=(0.4, 0.3, 0.25),   # отвод GND-площадки к переходу на плоскость: коротко и широко
)
DT = 10.0             # допустимый перегрев трассы, °C
DROP_FRAC = 0.02      # допустимое падение напряжения на питании: 2 % номинала (USB: 5 В ±5 %)
WMAX = 1.0            # шире не ведём: дальше нужен полигон, а не трасса
RHO = 1.72e-8         # Ом·м
OZ_MM = 0.035         # 1 oz

# Токи цепей по умолчанию (если в конфиге таблица не задана), А. Первое совпадение выигрывает.
CURRENTS = [
    (r'^VBUS', 1.0),
    (r'^\+?5V', 1.0),
    (r'^\+?3V3', 0.3),
]
NOMINAL_V = [(r'5V|VBUS', 5.0), (r'3V3', 3.3), (r'1V8', 1.8), (r'12V', 12.0)]

# Группы шин: цепи, которые надо вести параллельно на одном шаге.
BUS_RE = re.compile(r'^(?!Net-\()(.*?)(\d+|_[RGB]|[+-])$')


def ipc_width(I, t_mm, inner):
    """ширина, мм, по IPC-2221 для тока I, А и толщины меди t, мм"""
    if I <= 0: return 0.0
    k = 0.024 if inner else 0.048
    area_mil2 = (I / (k * DT ** 0.44)) ** (1 / 0.725)
    return area_mil2 / (t_mm / 0.0254) * 0.0254


def drop_width(I, length_mm, t_mm, vmax):
    if I <= 0 or vmax <= 0: return 0.0
    return RHO * (length_mm * 1e-3) * I / (t_mm * 1e-3 * vmax) * 1e3


def ceil_to(x, step=0.05):
    return round(math.ceil(x / step - 1e-9) * step, 3)


class Rules:
    def __init__(self, pcb_path, cfg=None):
        import config as _cfg
        self.cfg = cfg if cfg is not None else _cfg.load()
        self.v = dict(DEFAULTS)
        txt = open(pcb_path, encoding='utf-8').read()
        root = parse(txt)[0]
        self.bbox = self._edge_bbox(root)
        self.cu_t, self.stack_from_board = self._stackup(root)
        self._project(pcb_path)
        for k, val in self.cfg['rules'].items(): self.v[k] = val
        self.planes = self._planes(root)
        self.diel = self._dielectric()
        self._pairs()

    # ------------------------------------------------------------ board
    @staticmethod
    def _edge_bbox(root):
        xs, ys = [], []
        for kind in ('gr_line', 'gr_rect', 'gr_arc', 'gr_poly'):
            for e in find(root, kind):
                lay = find1(e, 'layer')
                if not lay or lay[1] != 'Edge.Cuts': continue
                for key in ('start', 'end', 'mid'):
                    p = find1(e, key)
                    if p: xs.append(float(p[1])); ys.append(float(p[2]))
                pts = find1(e, 'pts')
                if pts:
                    for xy in find(pts, 'xy'): xs.append(float(xy[1])); ys.append(float(xy[2]))
        if not xs: raise ValueError('на плате нет контура Edge.Cuts')
        return min(xs), min(ys), max(xs), max(ys)

    def _stackup(self, root):
        cc = self.cfg['stackup']['copper']
        t = {i: cc.get(n, OZ_MM) for i, n in enumerate(CU_NAMES)}
        setup = find1(root, 'setup')
        st = find1(setup, 'stackup') if setup else None
        found = False
        if st:
            for lay in find(st, 'layer'):
                ty = find1(lay, 'type'); th = find1(lay, 'thickness')
                if ty and ty[1] == 'copper' and th and lay[1] in CU_NAMES:
                    t[CU_NAMES.index(lay[1])] = float(th[1]); found = True
        return t, found

    def _planes(self, root):
        pl = self.cfg['stackup']['planes']
        if pl: return {CU_NAMES.index(n) for n in pl}
        zones = {}
        for z in find(root, 'zone'):
            lay = find1(z, 'layer'); net = find1(z, 'net')
            if lay and lay[1] in CU_NAMES[1:3]:
                zones.setdefault(CU_NAMES.index(lay[1]), set()).add(net[1] if net else '')
        return {l for l, nets in zones.items() if nets == {'GND'}}

    def _dielectric(self):
        out = {}
        for d in self.cfg['stackup']['dielectric']:
            a, b = (CU_NAMES.index(x) for x in d['between'])
            out[(min(a, b), max(a, b))] = (d['h'], d['er'])
        return out

    def ref_plane(self, l):
        """Опорная плоскость слоя l: (h, εr) или None. Плоскость считается опорной, только если она —
        ближайший по толщине диэлектрика слой меди: иначе обратный ток пойдёт по ближнему сигнальному слою
        (например, In2 в стеке 7628: до In1 1.065 мм, до B — 0.21 мм)."""
        nb = []
        for m in (l - 1, l + 1):
            d = self.diel.get((min(l, m), max(l, m)))
            if 0 <= m < 4 and d: nb.append((d[0], m, d))
        if not nb: return None
        h, m, d = min(nb)
        return d if m in self.planes else None

    def _project(self, pcb_path):
        pro = os.path.splitext(pcb_path)[0] + '.kicad_pro'
        self.classes = {}
        if not os.path.exists(pro): return
        try:
            j = json.load(open(pro, encoding='utf-8'))
        except (ValueError, OSError):
            return
        r = j.get('board', {}).get('design_settings', {}).get('rules', {})
        if r.get('min_copper_edge_clearance'): self.v['edge'] = max(self.v['edge'], r['min_copper_edge_clearance'])
        if r.get('min_clearance'): self.v['clearance'] = max(self.v['clearance'], r['min_clearance'])
        if r.get('min_track_width'): self.v['track_min'] = max(self.v['track_min'], r['min_track_width'])
        for c in j.get('net_settings', {}).get('classes', []):
            self.classes[c.get('name')] = c
            if c.get('name') == 'Default':
                for k_src, k_dst in (('clearance', 'clearance'), ('track_width', 'track'),
                                     ('via_diameter', 'via_d'), ('via_drill', 'via_drill')):
                    if c.get(k_src): self.v[k_dst] = max(self.v[k_dst], c[k_src])

    # ------------------------------------------------------------ nets
    def current(self, net):
        pw = self.cfg['power']
        loads = self.cfg.lookup(pw['loads'], net)
        if loads: return float(sum(loads.values()))
        I = self.cfg.lookup(pw['currents'], net)
        if I is not None: return float(I)
        if pw['currents']: return 0.0          # таблица задана — остальные цепи сигнальные
        for pat, I in CURRENTS:
            if re.search(pat, net): return I
        return 0.0

    def nominal(self, net):
        V = self.cfg.lookup(self.cfg['power']['nominal'], net)
        if V is not None: return float(V)
        for pat, V in NOMINAL_V:
            if re.search(pat, net): return V
        return 0.0

    def is_power(self, net):
        return self.current(net) >= 0.3

    def is_fast(self, net):
        return self.cfg.match(net, self.cfg['nets']['fast'])

    def fast_layers(self):
        """слои трассировки, у которых соседний по стеку слой — плоскость"""
        return {l for l in (0, 2, 3) if self.ref_plane(l) is not None}

    def widths(self, net, span):
        """Ступени ширины: [{layer: ширина, мм}, ...] от целевой к минимально допустимой.
        None — слой для цепи запрещён (по нагреву нужна медь шире WMAX, или у быстрой цепи нет там опорной плоскости).
        span — полупериметр bbox площадок цепи, мм: оценка пути от источника до дальней нагрузки.
        Нагрузки распределены вдоль цепи, поэтому падение ≈ I·R/2."""
        I = self.current(net)
        base = self.v['track']
        ok = self.fast_layers() if self.is_fast(net) else {0, 2, 3}
        def mask(d): return {l: (w if l in ok else None) for l, w in d.items()}
        if I <= 0:
            return [mask({l: base for l in (0, 2, 3)}), mask({l: self.v['track_min'] for l in (0, 2, 3)})]
        V = self.nominal(net); L = 1.2 * span
        target, thermal = {}, {}
        for l in (0, 2, 3):
            t = self.cu_t[l]
            wt = ceil_to(max(base, ipc_width(I, t, inner=l in (1, 2))))
            wd = ceil_to(drop_width(I / 2, L, t, DROP_FRAC * V)) if V else 0.0
            thermal[l] = wt if wt <= WMAX else None
            target[l] = None if thermal[l] is None else min(max(wt, wd), WMAX)
        steps = [mask(target)]
        if thermal != target: steps.append(mask(thermal))
        # последняя ступень — обычная сигнальная ширина: лучше проведённая и отмеченная в отчёте цепь,
        # чем неразведённая; такие места надо расширить вручную или полигоном
        steps.append(mask({l: base for l in (0, 2, 3)}))
        return steps

    # ------------------------------------------------------------ diff pairs
    def _pairs(self):
        """{net: (partner, info)} — ширина/зазор под целевой импеданс по слою с опорной плоскостью"""
        self.pair = {}; self.pair_info = []
        for item in self.cfg['nets']['diff_pairs']:
            a, b = item[0], item[1]; target = float(item[2]) if len(item) > 2 else 90.0
            layers = sorted(self.fast_layers()) or [0]
            l0 = layers[0]
            h, er = self.ref_plane(l0) or (0.2104, 4.4)
            w, s, z = impedance.design_pair(target, h, er, self.cu_t[l0],
                                            w_min=max(self.v['track_min'], 0.1),
                                            s_min=self.cfg['nets']['pair_gap_min'] or self.v['clearance'])
            info = dict(p=a, n=b, target=target, w=w, s=s, z=z, layer=l0, h=h, er=er,
                        ps_mm=impedance.ps_per_mm(er), layers=layers)
            self.pair[a] = (b, info); self.pair[b] = (a, info); self.pair_info.append(info)

    def report(self, net, segs):
        """падение напряжения при последовательном прохождении всех отрезков (верхняя оценка)"""
        I = self.current(net)
        if I <= 0: return None
        R = sum(RHO * (math.hypot(t[4] - t[2], t[5] - t[3]) * 1e-3) / (t[6] * 1e-3 * self.cu_t[t[1]] * 1e-3)
                for t in segs)
        wmin = min((t[6] for t in segs), default=0)
        return dict(I=I, R_mohm=R * 1e3, drop_mV=R * I * 1e3, w_min=wmin)


def bus_groups(nets, exclude=()):
    g = {}
    for n in nets:
        if n in exclude: continue
        m = BUS_RE.match(n)
        if m and m.group(1): g.setdefault(m.group(1), []).append(n)
    return {k: sorted(v) for k, v in g.items() if len(v) >= 2}
