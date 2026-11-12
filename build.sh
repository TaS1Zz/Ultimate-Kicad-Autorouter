#!/bin/sh
# Linux/macOS: ядро A* как разделяемая библиотека (GCC или Clang; без них — python -m pip install ziglang)
cd "$(dirname "$0")"
OUT=libastar.so; [ "$(uname)" = "Darwin" ] && OUT=libastar.dylib
if command -v cc >/dev/null 2>&1; then
    cc -O2 -std=c99 -shared -fPIC -o "$OUT" astar.c && echo "$OUT собран"
else
    python3 -m ziglang cc -O2 -std=c99 -shared -fPIC -o "$OUT" astar.c && echo "$OUT собран (zig cc)"
fi
