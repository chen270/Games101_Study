#!/bin/bash -e

mkdir -p build
cd build

cmake -G "Visual Studio 16 2019" -A x64 ../

# cmake --build . --config Release # 编译 Realse
# cmake --build . --config Debug