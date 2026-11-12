"""Установить плагин в KiCad: в папку плагинов кладётся загрузчик, указывающий на этот репозиторий.

    python tools/install_kicad_plugin.py              # найдёт папку плагинов последней версии KiCad
    python tools/install_kicad_plugin.py <папка>      # или указать её явно

После установки перезапустите редактор плат (или «Обновить плагины»). Python для трассировки задаётся
переменной UKA_PYTHON (в нём должны стоять зависимости из requirements.txt), иначе берётся python из PATH.
"""
import glob, os, sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def plugin_dir():
    home = os.path.expanduser('~')
    cands = glob.glob(os.path.join(home, 'Documents', 'KiCad', '*', 'scripting', 'plugins')) + \
            glob.glob(os.path.join(home, '.local', 'share', 'kicad', '*', 'scripting', 'plugins')) + \
            glob.glob(os.path.join(home, 'Library', 'Application Support', 'kicad', '*', 'scripting', 'plugins'))
    if cands:
        return sorted(cands, key=lambda p: [int(x) if x.isdigit() else 0 for x in p.split(os.sep)[-3].split('.')])[-1]
    vers = sorted(glob.glob(os.path.join(home, 'Documents', 'KiCad', '*')))
    if vers: return os.path.join(vers[-1], 'scripting', 'plugins')
    raise SystemExit('не нашёл папку плагинов KiCad — укажите её аргументом')


def main():
    d = sys.argv[1] if len(sys.argv) > 1 else plugin_dir()
    os.makedirs(d, exist_ok=True)
    dst = os.path.join(d, 'ultimate_kicad_autorouter.py')
    with open(dst, 'w', encoding='utf-8') as f:
        f.write('# загрузчик Ultimate Kicad Autorouter (сгенерирован tools/install_kicad_plugin.py)\n'
                'import os, sys\n'
                'os.environ.setdefault("UKA_HOME", %r)\n'
                'sys.path.insert(0, os.path.join(os.environ["UKA_HOME"], "kicad_plugin"))\n'
                'import ultimate_autorouter  # noqa: E402,F401\n' % ROOT)
    print('установлено:', dst)


if __name__ == '__main__':
    main()
