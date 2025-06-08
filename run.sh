#!/bin/bash

# Перейти в директорію скрипта (корінь проєкту)
cd "$(dirname "$0")"

# Запуск виконуваного файлу з абсолютним шляхом
./cmake-build-release/src/orca-slicer "$@"
