# Документация Ultimate Kicad Autorouter

Автотрассировщик 4-слойных плат KiCad: трассы только под 0/45/90°, согласование перегрузки (PathFinder),
шины и матрицы, дифпары под импеданс, ширина по току, полигоны питания с расчётом падения напряжения
и проверка результата DRC самого KiCad.

| Раздел | О чём |
|---|---|
| [Возможности](features.md) | Что умеет трассировщик и где его границы |
| [Установка и запуск](usage.md) | Сборка ядра, команды, выходные файлы, отчёт |
| [Конфигурация](configuration.md) | Справочник `boards/*.toml`, правила из платы, параметры поиска |
| [Как это работает](algorithms.md) | Этапы, A*, PathFinder, шины, дифпары, ширины, падение напряжения, DRC в цикле |
| [Плагин KiCad](plugin.md) | Кнопка в редакторе плат: установка и работа |
| [Разработка](development.md) | Устройство кода, тесты, CI, как добавить свою плату |

Быстрый старт:

```bash
pip install -r requirements.txt ziglang
build.bat                                          # Linux/macOS: sh build.sh
python route_board.py boards/demo.toml out/demo
```

Результат — `out/demo_routed.kicad_pcb` и отчёт `out/demo_report.md`.
