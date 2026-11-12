import os, sys
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
os.chdir(ROOT)


def pytest_configure(config):
    config.addinivalue_line('markers', 'slow: полная трассировка тестовой платы (~1–2 мин)')
