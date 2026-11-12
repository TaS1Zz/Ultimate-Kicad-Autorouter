@echo off
chcp 65001 >nul
rem Сборка ядра A* (astar.dll) для Windows x64.
rem Подойдёт любой из компиляторов: GCC (MinGW-w64 / MSYS2), Clang, MSVC (cl), либо zig из pip:
rem     python -m pip install ziglang
cd /d %~dp0
where gcc >nul 2>nul && (gcc -O2 -std=c99 -shared -o astar.dll astar.c && echo astar.dll собран GCC & goto :eof)
where clang >nul 2>nul && (clang -O2 -std=c99 -shared -o astar.dll astar.c && echo astar.dll собран Clang & goto :eof)
where cl >nul 2>nul && (cl /nologo /O2 /LD astar.c /Fe:astar.dll && echo astar.dll собран MSVC & goto :eof)
python -m ziglang cc -O2 -std=c99 -shared -target x86_64-windows-gnu -o astar.dll astar.c && echo astar.dll собран zig cc & goto :eof
echo Нет компилятора C. Установите: python -m pip install ziglang  — и запустите build.bat ещё раз.
