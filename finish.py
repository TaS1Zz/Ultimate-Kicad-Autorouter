"""Только постобработка и запись платы по сохранённому состоянию трассировки (без повторной трассировки).
Запуск: python finish.py boards/demo.toml out/demo [--no-drc]"""
import sys, pickle
import config, pipeline
from route2 import Router2

cfg = config.load(sys.argv[1]); out = sys.argv[2]
R = Router2(cfg.pcb(), cfg)
st = pickle.load(open(out + '.state.pkl', 'rb'))
R.tracks, R.vias, R.anchors, R.fan, R.fanrec, R.fail, R.warn = (st[k] for k in ('tracks', 'vias', 'anchors', 'fan', 'fanrec', 'fail', 'warn'))
R.rebuild()
pipeline.finish_and_check(R, cfg, out, drc='--no-drc' not in sys.argv, state=st)
