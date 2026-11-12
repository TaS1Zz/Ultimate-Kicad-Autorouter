"""Плагин KiCad без интерфейса: Python из комплекта KiCad загружает плату, плагин разводит её внешним Python
и переносит результат, затем kicad-cli проверяет плату. Пропускается, если KiCad не установлен."""
import glob, os, shutil, subprocess, sys, textwrap, pytest
import pipeline


def kicad_python():
    c = sorted(glob.glob(r'C:\Program Files\KiCad\*\bin\python.exe'),
               key=lambda p: [int(x) for x in os.path.basename(os.path.dirname(os.path.dirname(p))).split('.') if x.isdigit()])
    for p in c[::-1] + ['/usr/bin/python3']:
        if os.path.exists(p) and subprocess.run([p, '-c', 'import pcbnew'], capture_output=True).returncode == 0:
            return p
    return None


@pytest.mark.slow
@pytest.mark.skipif(kicad_python() is None or pipeline.kicad_cli() is None, reason='нет KiCad')
def test_plugin_applies_routing(tmp_path):
    board = tmp_path / 'demo.kicad_pcb'
    shutil.copy('example/demo_board.kicad_pcb', board)
    script = tmp_path / 'run.py'
    script.write_text(textwrap.dedent('''
        import sys, os, tempfile
        sys.path.insert(0, os.path.join(%r, 'kicad_plugin'))
        import pcbnew, ultimate_autorouter as U
        p = %r
        b = pcbnew.LoadBoard(p)
        routed, rep, _ = U.run_router(p, tempfile.mkdtemp())
        n = U.apply_routed(b, routed)
        pcbnew.SaveBoard(p, b)
        open(p + '.ok', 'w').write(repr(n))
    ''' % (os.getcwd(), str(board))), encoding='utf-8')
    env = dict(os.environ, UKA_PYTHON=sys.executable)
    subprocess.run([kicad_python(), str(script)], env=env, capture_output=True, timeout=900)
    assert os.path.exists(str(board) + '.ok')
    rpt = tmp_path / 'drc.json'
    subprocess.run([pipeline.kicad_cli(), 'pcb', 'drc', '--format', 'json', '--refill-zones', '--severity-error',
                    '-o', str(rpt), str(board)], capture_output=True)
    import json
    j = json.load(open(rpt, encoding='utf-8'))
    types = {v['type'] for v in j['violations']}
    assert not j.get('unconnected_items')
    assert not types & {'clearance', 'via_diameter', 'track_dangling', 'shorting_items', 'hole_to_hole'}
