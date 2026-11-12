"""Ultimate Kicad Autorouter — полный цикл для любой платы по её конфигу.

    python route_board.py boards/demo.toml out/demo             # трассировка + запись + DRC KiCad + отчёт
    python route_board.py boards/demo.toml out/demo --route-only
    python route_board.py boards/demo.toml out/demo --threads 4 --no-drc

Этапы: переходы под открытыми площадками → fanout → питание (широко) → GND (отвод на плоскость у каждой
площадки) → дифпары → сигналы (PathFinder) → узаконивание → rip-up для остатка → постобработка → полигоны
питания → запись .kicad_pcb → DRC KiCad в цикле → отчёт (качество, падение напряжения по меди, перекос пар).
"""
import argparse, os, sys, time, pickle, math
import config
from route2 import Router2, RL
import route as G


def log(*a):
    print(*a, flush=True)


def driver_key(R, n):
    """порядок цепей внутри шины: по положению вывода ведущей микросхемы вдоль её стороны"""
    ps = [p for p in R.pads if p.net == n and p.ref[:1] == 'U'] or [p for p in R.pads if p.net == n]
    p = ps[0]
    return (p.ref, p.y if p.w > p.h else p.x)


def route(cfg, out, threads=1):
    t = time.time()
    R = Router2(cfg.pcb(), cfg)
    ru = R.rules
    log('плата %.2f..%.2f x %.2f..%.2f мм, зазор %.2f, край %.2f, переход %.2f/%.2f' %
        (G.X0, G.X1, G.Y0, G.Y1, G.CLR, G.EDGE, G.VIA_D, G.VIA_DRILL))
    log('медь, мм:', {G_name: ru.cu_t[l] for l, G_name in enumerate(['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu'])},
        '| плоскости:', sorted(ru.planes), '| слои быстрых цепей:', sorted(ru.fast_layers()))
    power = sorted([n for n in R.netid if ru.is_power(n)], key=lambda n: -ru.current(n))
    for n in power:
        log('  %-9s %.2f А  ширины %s' % (n, ru.current(n), R.steps(n)))
    for info in ru.pair_info:
        log('  пара %s/%s: w=%.3f s=%.3f мм → Zdiff %.1f Ом (цель %.0f) на %s' %
            (info['p'], info['n'], info['w'], info['s'], info['z'], info['target'], ['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu'][info['layer']]))
    log('шины:', R.groups)

    # переходы под открытыми GND-площадками микросхем
    for (x, y) in config.thermal_vias(cfg, R.pads, G.VIA_D):
        R.add_via('GND', x, y, 'fixed')
        for l in range(4): R.anchors.setdefault('GND', set()).add((l, G.gy(y), G.gx(x)))
    # питающие цепи — с номинальным напряжением (VBUS, +3V3, …): их выводам нужен переход к плоскости/слою питания
    plan = config.fanout_plan(cfg, R.pads, power_nets={n for n in R.netid if ru.nominal(n) > 0})
    bad = {}
    for ref, nums in plan.items():
        b = R.fanout(ref, nums)
        if b: bad[ref] = b
    log('fanout', {k: len(v) if v else 'все' for k, v in plan.items()}, 'не вывелись', bad, round(time.time() - t, 1))

    # порядок — от самых ограниченных цепей: дифпары (один слой, связанная пара), затем питание (широко,
    # но может уйти на B), затем отводы GND на плоскость
    pairs = []
    for info in ru.pair_info:
        for n in (info['p'], info['n']):
            R.route_net(n); pairs.append(n)
    if pairs: log('дифпары', round(time.time() - t, 1), [f for f in R.fail if f[0] in pairs])
    for n in power: R.route_net(n)
    R.route_net('GND', gnd=True)
    log('питание и GND', round(time.time() - t, 1), R.fail)

    # сигналы: группы шин подряд (порядок выводов), длинные цепи раньше коротких
    done = set(power) | {'GND'} | set(pairs)
    order = [n for n in cfg['nets']['order_first'] if n in R.netid and n not in done]
    for g, members in sorted(R.groups.items(), key=lambda kv: -max(R.span(n) for n in kv[1])):
        order += [n for n in sorted(members, key=lambda n: driver_key(R, n)) if n not in done and n not in order]
    rest = [n for n in R.netid if n not in done and n not in order]
    order += sorted([n for n in rest if R.span(n) > 60], key=R.span, reverse=True)
    order += sorted([n for n in rest if R.span(n) <= 60 and n not in order], key=R.span)
    its, left = R.negotiate(order, cfg['route']['negotiate_iters'], log=log, threads=threads)
    log('PathFinder: %d проходов, конфликтов осталось у %d цепей, %.1f с' % (its, len(left), time.time() - t))

    # узаконивание: точная проверка зазоров; конфликтующие цепи — переразвести жёстко
    for k in range(3):
        viol = R.violations()
        if not viol: break
        nets = sorted({b for a, b, _, _ in viol if b not in power and b != 'GND' and b not in pairs} |
                      {a for a, b, _, _ in viol if a not in power and a != 'GND' and a not in pairs})
        log('  зазоры после согласования: %d, переразводка %s' % (len(viol), nets))
        for n in nets: R.rip(n)
        R.rebuild()
        for n in nets: R.route_net(n)
    rip_up(R, list(power) + list(pairs), pairs)
    log('итог трассировки', round(time.time() - t, 1), 'с; неразведено', R.fail)
    if R.warn: log('предупреждения по ширине:', R.warn)
    state = dict(tracks=R.tracks, vias=R.vias, fail=R.fail, warn=R.warn, anchors=R.anchors, fan=R.fan, fanrec=R.fanrec,
                 power=power, pairs=pairs, seconds=time.time() - t)
    pickle.dump(state, open(out + '.state.pkl', 'wb'))
    tr, vi = R.export()
    pickle.dump((tr, vi, R.fail, R.warn), open(out + '.pkl', 'wb'))
    return R, state


def rip_up(R, power, pairs, max_it=60):
    """запасной путь для того, что не развела перегрузка: rip-up мешающих с откатом, если питание сузилось бы.
    power — защищённые цепи (питание и дифпары): их снимают только на второй попытке, если мешают только они"""
    tries = {}; unres = []
    for it in range(max_it):
        if not R.fail: break
        name, ref, num = R.fail[0]
        if name == 'GND': unres.append(R.fail.pop(0)); continue
        key = (name, ref, num); tries[key] = tries.get(key, 0) + 1
        if tries[key] > 4: log('  оставлено:', key); unres.append(R.fail.pop(0)); continue
        pad = [p for p in R.pads if p.ref == ref and p.num == num][0]
        hit = R.blockers(name, pad)
        if hit is not None and not hit:
            R.rip(name); R.rebuild(); R.route_net(name)
            R.fail = [f for f in R.fail if f[0] != name] + [f for f in R.fail if f[0] == name]
            continue
        if hit is None: log('  нет пути даже сквозь чужие трассы:', key); unres.append(R.fail.pop(0)); continue
        victims = ([n for n in hit if n not in power and n != 'GND'] or [n for n in hit if n == 'GND'] or list(hit)) \
            if tries[key] == 1 else ([n for n in hit if n != 'GND'] or list(hit))
        snap = ([list(x) if x else None for x in R.tracks], [list(x) if x else None for x in R.vias],
                {k: set(v) for k, v in R.anchors.items()}, list(R.fail), list(R.warn))
        log('  rip-up', key, '→', victims)
        for v in victims: R.rip(v)
        R.rip(name); R.rebuild()
        R.route_net(name)
        for v in [v for v in victims if v in power]: R.route_net(v)
        for v in [v for v in victims if v not in power]:
            if v == 'GND': R.route_net('GND', gnd=True)
            else: R.route_net(v)
        if any(w[0] in power and w[0] in victims and 'сигнальная' in w[3] for w in R.warn):
            R.tracks, R.vias, R.anchors, R.fail, R.warn = snap
            R.rebuild()
            R.fail = [f for f in R.fail if f[0] != name] + [f for f in R.fail if f[0] == name]
            log('   откат: питание сузилось бы до сигнальной ширины')
    gl = [u for u in unres if u[0] == 'GND']
    if gl:
        R.fail = []; R.route_net('GND', gnd=True)
        unres = [u for u in unres if u[0] != 'GND'] + list(R.fail)
    R.fail = unres


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('config'); ap.add_argument('out')
    ap.add_argument('--route-only', action='store_true')
    ap.add_argument('--no-drc', action='store_true')
    ap.add_argument('--threads', type=int, default=None)
    a = ap.parse_args()
    cfg = config.load(a.config)
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    threads = a.threads if a.threads is not None else (cfg['route']['threads'] or os.cpu_count() or 1)
    R, state = route(cfg, a.out, threads)
    if a.route_only: return
    import pipeline
    pipeline.finish_and_check(R, cfg, a.out, drc=not a.no_drc, state=state)


if __name__ == '__main__':
    main()
