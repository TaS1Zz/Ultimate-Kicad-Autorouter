"""После трассировки: постобработка → полигоны питания → запись .kicad_pcb → DRC KiCad в цикле → отчёт.

DRC в цикле: kicad-cli заливает зоны и проверяет плату; замечания, которые можно исправить трассировкой,
исправляются и плата проверяется снова (до drc.iters раз, сохраняется лучший вариант):
  * starved_thermal — у GND-площадки не хватает мостиков терморазгрузки: чужие трассы отодвигаются от неё
    (локальный отступ) и переразводятся;
  * clearance / track_dangling / via_dangling / unconnected_items — цепи из замечания переразводятся.
Замечания по шелкографии и библиотеке футпринтов к трассировке не относятся и только перечисляются в отчёте.
"""
import json, math, os, shutil, subprocess, sys, time, glob
from merge import merge
from post import postprocess
from zones import add_zones
import power, diffpair, metrics, check

ROUTING_DRC = {'clearance', 'hole_clearance', 'copper_edge_clearance', 'track_dangling', 'via_dangling',
               'unconnected_items', 'starved_thermal', 'shorting_items', 'tracks_crossing', 'track_width',
               'annular_width', 'via_diameter', 'diff_pair_gap_out_of_range'}


def log(*a): print(*a, flush=True)


def kicad_cli():
    def ver(p):          # «10.0» новее «8.0»: сравнивать числа, а не строки
        try: return tuple(int(x) for x in os.path.basename(os.path.dirname(os.path.dirname(p))).split('.'))
        except ValueError: return (0,)
    for c in [os.environ.get('KICAD_CLI'), shutil.which('kicad-cli')] + \
             sorted(glob.glob(r'C:\Program Files\KiCad\*\bin\kicad-cli.exe'), key=ver, reverse=True) + \
             ['/usr/bin/kicad-cli', '/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli']:
        if c and os.path.exists(c): return c
    return None


def write_board(R, cfg, out):
    tr, vi = R.export()
    tr = merge(list(tr), list(vi))
    tr, vi = postprocess(tr, vi, R.pads)
    zs, zinfo = power.zones(R, tr, cfg)
    dst = out + '_routed.kicad_pcb'
    R.emit(cfg.pcb(), dst + '.tmp', [tuple(t[:7]) for t in tr], [tuple(v[:3]) for v in vi], zones=zs)
    txt = add_zones(open(dst + '.tmp', encoding='utf-8').read())
    open(dst, 'w', encoding='utf-8').write(txt); os.remove(dst + '.tmp')
    pair_rules(R, dst)
    return dst, tr, vi, zinfo


def pair_rules(R, dst):
    """если зазор пары меньше общего — отдельное правило для KiCad (.kicad_dru рядом с платой)"""
    lines = []
    for i in R.rules.pair_info:
        if i['s'] < R.rules.v['clearance'] - 1e-6:
            lines.append('(rule "pair %s/%s"\n  (condition "(A.NetName == \'%s\' && B.NetName == \'%s\') || '
                         '(A.NetName == \'%s\' && B.NetName == \'%s\')")\n  (constraint clearance (min %.3fmm)))'
                         % (i['p'], i['n'], i['p'], i['n'], i['n'], i['p'], i['s']))
    if lines:
        open(os.path.splitext(dst)[0] + '.kicad_dru', 'w', encoding='utf-8').write('(version 1)\n' + '\n'.join(lines) + '\n')


def kicad_drc(path):
    cli = kicad_cli()
    if not cli: return None
    rpt = os.path.splitext(path)[0] + '_drc.json'
    subprocess.run([cli, 'pcb', 'drc', '--format', 'json', '--refill-zones', '--save-board', '--severity-error',
                    '--severity-warning', '-o', rpt, path], capture_output=True)
    if not os.path.exists(rpt): return None
    j = json.load(open(rpt, encoding='utf-8'))
    v = j.get('violations', []) + [dict(x, type='unconnected_items') for x in j.get('unconnected_items', [])]
    return v


def summarize(v):
    out = {}
    for x in v: out[x['type']] = out.get(x['type'], 0) + 1
    return out


def _routing_count(v):
    return sum(1 for x in v if x['type'] in ROUTING_DRC)


def _fix(R, v, protected):
    """исправить то, что можно трассировкой; вернуть число действий"""
    from route_board import rip_up
    acts = 0; nets = set()
    R.keep_extra = getattr(R, 'keep_extra', {})
    for x in v:
        items = x.get('items', [])
        if x['type'] == 'starved_thermal':
            for it in items:
                p = it.get('pos', {})
                for q in R.pads:
                    if q.net == 'GND' and abs(q.x - p.get('x', 1e9)) < 0.05 and abs(q.y - p.get('y', 1e9)) < 0.05:
                        k = R.pkey(q); R.keep_extra[k] = R.keep_extra.get(k, 0.0) + 0.2; acts += 1
                        r = max(q.w, q.h) / 2 + 1.2
                        for t in R.tracks:
                            if t is None or t[0] in protected or t[0] == 'GND' or t[1] not in (0, 3): continue
                            from shapely.geometry import LineString, Point
                            if LineString([(t[2], t[3]), (t[4], t[5])]).distance(Point(q.x, q.y)) < r: nets.add(t[0])
        elif x['type'] in ROUTING_DRC:
            for it in items:
                d = it.get('description', '')
                for n in R.netid:
                    if '[%s]' % n in d and n not in protected and n != 'GND': nets.add(n)
    if not acts and not nets: return 0
    for n in nets: R.rip(n)
    R.rebuild()
    for n in sorted(nets, key=R.span, reverse=True): R.route_net(n)
    rip_up(R, list(protected), [])
    return acts + len(nets)


def finish_and_check(R, cfg, out, drc=True, state=None):
    t0 = time.time()
    protected = set(n for n in R.netid if R.rules.is_power(n)) | set(R.rules.pair)
    best = None
    iters = cfg['drc']['iters'] if (drc and cfg['drc']['loop']) else 0
    for it in range(iters + 1):
        dst, tr, vi, zinfo = write_board(R, cfg, out)
        v = kicad_drc(dst) if drc else None
        if v is None:
            best = (0, dst, tr, vi, zinfo, None); break
        rc = _routing_count(v)
        log('DRC KiCad, проход %d: по трассировке %d, всего %d %s' % (it + 1, rc, len(v), summarize(v)))
        if best is None or rc < best[0]:
            best = (rc, dst, tr, vi, zinfo, v)
            shutil.copy(dst, dst + '.best')
        if rc == 0 or it == iters: break
        if not _fix(R, v, protected): break
    rc, dst, tr, vi, zinfo, v = best
    if os.path.exists(dst + '.best'):
        shutil.move(dst + '.best', dst)
    rep = report(R, cfg, out, dst, tr, vi, zinfo, v, state)
    log('отчёт: %s (%.1f с)' % (out + '_report.md', time.time() - t0))
    return rep


def report(R, cfg, out, dst, tr, vi, zinfo, v, state):
    m = metrics.compute(tr, vi, R.pads, R.groups)
    viol, bad = check.check(tr, vi, R.pads, R.rules)
    drop = power.drop_report(dst, R, cfg) if v is not None else {}
    pairs = diffpair.report(R, tr, vi)
    widths = {}
    for n in R.netid:
        if R.rules.current(n) > 0 and R.rules.is_power(n):
            ws = [t[6] for t in tr if t[0] == n]
            if ws: widths[n] = dict(I=R.rules.current(n), target=R.steps(n)[0], w_max=max(ws), w_min=min(ws))
    rep = dict(board=os.path.basename(cfg.pcb()), routed_board=os.path.basename(dst), metrics=m,
               check=dict(violations=len(viol), unconnected=[b[0] for b in bad]),
               drc=None if v is None else dict(total=len(v), routing=_routing_count(v), by_type=summarize(v)),
               power_widths=widths, power_polygons=zinfo, ir_drop=drop, diff_pairs=pairs,
               fails=list(R.fail), warnings=list(R.warn),
               route_seconds=None if not state else round(state.get('seconds', 0), 1))
    json.dump(rep, open(out + '_report.json', 'w', encoding='utf-8'), ensure_ascii=False, indent=1, default=str)
    L = ['# Отчёт трассировки: %s' % rep['board'], '',
         '| | |', '|---|---|',
         '| Неразведено | %d |' % len(R.fail),
         '| Нарушений зазоров (своя проверка) | %d |' % len(viol),
         '| Неподключённых цепей (своя проверка) | %d |' % len(bad),
         '| DRC KiCad: по трассировке / всего | %s |' % ('—' if v is None else '%d / %d' % (rep['drc']['routing'], rep['drc']['total'])),
         '| Отрезков / переходов / длина | %d / %d / %.0f мм |' % (m['segments'], m['vias'], m['length_mm']),
         '| Не под 0/45/90° | %d |' % m['off_angle'],
         '| Изломов / «ступенек» < 0,5 мм | %d / %d |' % (m['bends'], m['jogs']),
         '| Обход (длина / MST) средний | %.2f |' % m['detour_mean'],
         '| Время трассировки | %s с |' % rep['route_seconds'], '']
    if rep['drc']: L += ['DRC KiCad по типам: ' + ', '.join('%s %d' % kv for kv in sorted(rep['drc']['by_type'].items())), '']
    if m['bus_parallel']:
        L += ['## Шины — доля длины, идущей параллельно соседям', ''] + \
             ['* %s: %d %%' % kv for kv in sorted(m['bus_parallel'].items())] + ['']
    if widths:
        L += ['## Питание', '', '| Цепь | Ток, А | Ширина, мм (мин…макс) | Падение по меди |', '|---|---|---|---|']
        for n, w in sorted(widths.items()):
            d = drop.get(n)
            ds = '%.0f мВ (%.2f %%)' % (d['max_mV'], d['pct']) if d and d.get('pct') is not None else ('%.0f мВ' % d['max_mV'] if d else '—')
            L.append('| %s | %.2f | %.2f…%.2f | %s |' % (n, w['I'], w['w_min'], w['w_max'], ds))
        if zinfo: L += ['', 'Полигоны: ' + ', '.join('%s на %s (%.0f мм²)' % z for z in zinfo)]
        L.append('')
    if pairs:
        L += ['## Дифпары', '']
        for p in pairs:
            if 'note' in p: L.append('* %s/%s: %s' % (p['pair'] + (p['note'],))); continue
            L.append('* %s/%s: w=%.3f, s=%.3f мм → Zdiff %.1f Ом (цель %.0f); длины %s / %s мм; перекос %s мм (%s пс) %s'
                     % (p['pair'][0], p['pair'][1], p['w'], p['s'], p['zdiff'], p['target'], p['len_p'], p['len_n'],
                        p.get('skew_mm'), p.get('skew_ps'), '— в норме' if p.get('ok') else '— больше допуска'))
        L.append('')
    if R.fail: L += ['## Неразведено', ''] + ['* %s %s.%s' % f for f in R.fail] + ['']
    if R.warn: L += ['## Предупреждения', ''] + ['* %s %s.%s: %s' % w for w in R.warn] + ['']
    open(out + '_report.md', 'w', encoding='utf-8').write('\n'.join(L))
    return rep
