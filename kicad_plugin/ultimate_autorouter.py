"""Ultimate Kicad Autorouter — Action-плагин для редактора плат KiCad (8, 9, 10).

Установка: python tools/install_kicad_plugin.py — кладёт в папку плагинов KiCad загрузчик, который указывает
на этот репозиторий. Кнопка появляется на панели инструментов редактора плат (Инструменты → Внешние плагины).

Что делает кнопка:
  1. сохраняет плату;
  2. запускает трассировщик во внешнем Python (нужны numpy, shapely, scipy; путь — переменная UKA_PYTHON,
     иначе python3/python из PATH) с конфигом <плата>.toml рядом с платой, если он есть, иначе с настройками
     по умолчанию;
  3. переносит трассы, переходы и полигоны питания в открытую плату, заливает зоны и показывает отчёт.
"""
import os, sys, shutil, subprocess, tempfile

ROOT = os.environ.get('UKA_HOME') or os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def python_exe():
    return os.environ.get('UKA_PYTHON') or shutil.which('python3') or shutil.which('python') or 'python'


def config_for(board_path, workdir):
    toml = os.path.splitext(board_path)[0] + '.toml'
    if os.path.exists(toml): return toml
    cfg = os.path.join(workdir, 'board.toml')
    with open(cfg, 'w', encoding='utf-8') as f:
        f.write('[board]\nfile = %r\n' % board_path.replace('\\', '/'))
    return cfg


def run_router(board_path, workdir, drc=False):
    """трассировка во внешнем Python; возвращает (путь к разведённой плате, путь к отчёту, вывод)"""
    out = os.path.join(workdir, 'routed')
    cmd = [python_exe(), os.path.join(ROOT, 'route_board.py'), config_for(board_path, workdir), out]
    if not drc: cmd.append('--no-drc')
    p = subprocess.run(cmd, capture_output=True, text=True, encoding='utf-8', errors='replace', cwd=ROOT)
    if p.returncode != 0: raise RuntimeError(p.stdout[-3000:] + '\n' + p.stderr[-3000:])
    return out + '_routed.kicad_pcb', out + '_report.md', p.stdout


def apply_routed(board, routed_path):
    """скопировать трассы, переходы и зоны трассировщика из разведённой платы в открытую"""
    import pcbnew
    src = pcbnew.LoadBoard(routed_path)
    have = {(z.GetNetname(), z.GetLayer()) for z in board.Zones()}
    n_t = n_v = n_z = 0
    for t in src.GetTracks():
        net = board.FindNet(t.GetNetname())
        if t.GetClass() == 'PCB_VIA':
            # копия целиком: в KiCad 9+ размер перехода хранится в padstack, отдельные сеттеры его не задают
            v = t.Duplicate()
            if not hasattr(v, 'GetDrillValue'): v = pcbnew.Cast_to_PCB_VIA(v)
            if net: v.SetNet(net)
            board.Add(v); n_v += 1
        else:
            n = pcbnew.PCB_TRACK(board)
            n.SetStart(t.GetStart()); n.SetEnd(t.GetEnd()); n.SetWidth(t.GetWidth()); n.SetLayer(t.GetLayer())
            if net: n.SetNet(net)
            board.Add(n); n_t += 1
    for z in src.Zones():
        if (z.GetNetname(), z.GetLayer()) in have: continue
        nz = pcbnew.ZONE(board)
        nz.SetLayer(z.GetLayer()); nz.SetAssignedPriority(z.GetAssignedPriority())
        # контур — по вершинам (SetOutline забирает владение объектом: из Python это двойное освобождение)
        src_ol = z.Outline(); ol = nz.Outline()
        for k in range(src_ol.OutlineCount()):
            ch = src_ol.COutline(k); ol.NewOutline()
            for i in range(ch.PointCount()):
                pt = ch.CPoint(i); ol.Append(pt.x, pt.y)
        nz.SetZoneName(z.GetZoneName())
        nz.SetLocalClearance(z.GetLocalClearance()); nz.SetMinThickness(z.GetMinThickness())
        nz.SetPadConnection(z.GetPadConnection())
        nz.SetThermalReliefGap(z.GetThermalReliefGap()); nz.SetThermalReliefSpokeWidth(z.GetThermalReliefSpokeWidth())
        net = board.FindNet(z.GetNetname())
        if net: nz.SetNet(net)
        board.Add(nz); n_z += 1
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    return n_t, n_v, n_z


try:
    import pcbnew, wx

    class UltimateAutorouter(pcbnew.ActionPlugin):
        def defaults(self):
            self.name = 'Ultimate Kicad Autorouter'
            self.category = 'Routing'
            self.description = 'Октилинейная трассировка: PathFinder, шины, дифпары, ширина по току, DRC'
            self.show_toolbar_button = True
            ic = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'icon.png')
            self.icon_file_name = ic if os.path.exists(ic) else ''

        def Run(self):
            board = pcbnew.GetBoard()
            path = board.GetFileName()
            if not path:
                wx.MessageBox('Сначала сохраните плату.', 'Ultimate Kicad Autorouter'); return
            if any(True for _ in board.GetTracks()):
                if wx.MessageBox('На плате уже есть трассы. Трассировщик добавит свои поверх. Продолжить?',
                                 'Ultimate Kicad Autorouter', wx.YES_NO) != wx.YES: return
            pcbnew.SaveBoard(path, board)
            work = tempfile.mkdtemp(prefix='uka_')
            busy = wx.BusyInfo('Трассировка… (это может занять несколько минут)')
            try:
                routed, report, _ = run_router(path, work)
                n = apply_routed(board, routed)
            except Exception as e:
                del busy
                wx.MessageBox('Ошибка трассировки:\n%s' % e, 'Ultimate Kicad Autorouter'); return
            del busy
            pcbnew.Refresh()
            wx.MessageBox('Добавлено: отрезков %d, переходов %d, зон %d.\nОтчёт: %s\nПроверьте плату DRC.' % (n + (report,)),
                          'Ultimate Kicad Autorouter')

    if wx.GetApp() is not None:          # регистрировать только внутри KiCad (вне его register() падает)
        UltimateAutorouter().register()
except Exception:          # импорт вне KiCad (тесты, CLI) — только функции выше
    pass
