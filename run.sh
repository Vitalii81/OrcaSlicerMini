#!/bin/bash

# Перейти в директорію скрипта (корінь проєкту)
cd "$(dirname "$0")"

# Запуск виконуваного файлу з абсолютним шляхом
./build/src/orca-slicer "$@"
#./cmake-build-release/src/orca-slicer "$@"
