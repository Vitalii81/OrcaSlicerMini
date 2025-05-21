#!/bin/bash
set -e

# Шлях до RAM-диска
RAMDISK=~/ramdisk
SRC_DIR="$PWD"
BUILD_DIR="$RAMDISK/build"
PROJECT_DIR_NAME=$(basename "$SRC_DIR")

# 1. Створити RAM-диск (якщо ще нема)
if [ ! -d "$RAMDISK" ]; then
  echo "🧠 Створюю каталог RAM-диска у $RAMDISK..."
  mkdir -p "$RAMDISK"
fi

# Перевірити, чи вже змонтований
if ! mountpoint -q "$RAMDISK"; then
  echo "🧠 Монтую tmpfs у $RAMDISK (4G)..."
  sudo mount -t tmpfs -o size=4G tmpfs "$RAMDISK"
fi

# 2. Очистити та скопіювати проєкт
echo "📦 Копіюю проект у RAM-диск..."
rm -rf "$RAMDISK/$PROJECT_DIR_NAME"
rsync -a --exclude build "$SRC_DIR/" "$RAMDISK/$PROJECT_DIR_NAME/"

cd "$RAMDISK/$PROJECT_DIR_NAME"

# 3. Генеруємо CMake та збираємо
echo "🔧 Запускаю CMake..."
cmake -S . -B build -G Ninja -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=Release

echo "🚀 Збираю проект..."
cmake --build build -j$(nproc)

# 4. Копіюємо compile_commands.json назад
if [ -f build/compile_commands.json ]; then
  echo "📤 Копіюю compile_commands.json назад у $SRC_DIR"
  cp build/compile_commands.json "$SRC_DIR"
fi

rsync -a --delete ~/ramdisk/Slice3rCore/build/ ./build/

sudo umount ~/ramdisk

echo "✅ Готово. Зібрано в $BUILD_DIR"
