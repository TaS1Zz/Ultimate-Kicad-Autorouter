# Установка и запуск

## Что нужно

* Python 3.10+ и зависимости: `pip install -r requirements.txt` (numpy, shapely, scipy, matplotlib).
* C-компилятор для ядра A*. Без него: `pip install ziglang` — `build.bat` и `build.sh` используют его сами.
* KiCad 8+ с `kicad-cli` — для DRC и заливки зон. Без KiCad трассировка и своя проверка работают,
  DRC-этап пропускается (`--no-drc`). Путь к `kicad-cli` ищется в PATH, в переменной `KICAD_CLI` и в стандартных
  папках установки (берётся самая новая версия).

## Сборка ядра

```bash
build.bat        # Windows: zig → GCC (статически) → Clang → MSVC — что найдётся
sh build.sh      # Linux/macOS: cc, иначе zig
```

Получается `astar.dll` / `libastar.so` / `libastar.dylib` рядом с `route2.py`. В репозитории лежит готовый
`astar.dll` для Windows x64.

## Запуск

```bash
python route_board.py <конфиг.toml> <префикс выхода> [--route-only] [--no-drc] [--threads N]
```

| Ключ | Что делает |
|---|---|
| `--route-only` | Только трассировка: сохраняет состояние, плату не пишет |
| `--no-drc` | Записать плату и отчёт без `kicad-cli` (без заливки зон и DRC-цикла) |
| `--threads N` | Потоков для параллельной разводки независимых цепей; по умолчанию — `route.threads` из конфига или число ядер |

Пример: `python route_board.py boards/demo.toml out/demo`.

Отдельные шаги:

```bash
python finish.py boards/demo.toml out/demo [--no-drc]   # запись платы и DRC по сохранённому состоянию
python check.py out/demo.pkl example/demo_board.kicad_pcb boards/demo.toml   # зазоры и связность
python metrics.py out/demo.pkl example/demo_board.kicad_pcb                  # метрики качества
python viz.py out/demo.pkl example/demo_board.kicad_pcb 100 200 100 170 out/demo.png   # x0 x1 y0 y1
```

## Выходные файлы

| Файл | Что в нём |
|---|---|
| `<out>_routed.kicad_pcb` | Разведённая плата: трассы, переходы, полигоны питания, заливки GND на F/B (зоны залиты KiCad) |
| `<out>_report.md`, `<out>_report.json` | Отчёт |
| `<out>_routed_drc.json` | Последний отчёт DRC KiCad |
| `<out>.pkl` | Трассы и переходы до постобработки (для `check.py`, `metrics.py`, `viz.py`) |
| `<out>.state.pkl` | Полное состояние трассировщика (для `finish.py`) |
| `<out>_routed.kicad_dru` | Правило зазора внутри дифпары — только если он меньше общего зазора платы |

## Отчёт

Пример (тестовая плата):

```
| Неразведено | 0 |
| Нарушений зазоров (своя проверка) | 0 |
| Неподключённых цепей (своя проверка) | 0 |
| DRC KiCad: по трассировке / всего | 2 / 3 |
| Отрезков / переходов / длина | 271 / 71 / 1382 мм |
| Не под 0/45/90° | 0 |
| Изломов / «ступенек» < 0,5 мм | 115 / 26 |
| Обход (длина / MST) средний | 1.03 |

| VBUS | 1.31 | 0.20…0.55 | 37 мВ (0.74 %) |
* USB_D+/USB_D-: w=0.295, s=0.205 мм → Zdiff 90.0 Ом (цель 90); длины 51.07 / 51.17 мм; перекос 0.1 мм (0.6 пс) — в норме
```

Поля JSON: `metrics` (segments, vias, off_angle, bends, jogs, detour_mean, detour_worst, bus_parallel,
length_mm), `check` (violations, unconnected), `drc` (total, routing, by_type), `power_widths`, `power_polygons`,
`ir_drop` (по цепи: max_mV, pct, per_load, I, V), `diff_pairs` (w, s, zdiff, len_p, len_n, skew_mm, skew_ps, ok),
`fails`, `warnings`, `route_seconds`.

«Ступенька» — излом, у которого один из соседних отрезков короче 0,5 мм; часть их — законные подходы к
переходам и площадкам. «Обход» — длина трасс цепи, делённая на длину октилинейного минимального дерева
по её площадкам: 1,0 — идеально прямо.

## Тестовая плата

`example/demo_board.kicad_pcb` пересобирается Python из комплекта KiCad:

```bash
"C:\Program Files\KiCad\10.0\bin\python.exe" tools/make_demo_board.py example/demo_board.kicad_pcb
```

100 × 70 мм, 4 слоя, 47 футпринтов, 36 цепей — только стандартные библиотеки KiCad: MCU в LQFP-48, драйвер
светодиодов в QFN-32 с открытой площадкой, USB-C с дифпарой, LDO 3,3 В, матрица кнопок 2 × 4 с диодами,
шина из 8 светодиодов, разъём питания/SWD. Путь к библиотекам — переменная `KICAD_FOOTPRINTS`.
