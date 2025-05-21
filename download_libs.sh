#!/bin/bash
# Завантаження та збірка необхідних бібліотек OrcaSlicer у src/deps
set -e

mkdir -p src/deps
cd src/deps

# === Boost (коментовано, бо використовується system Boost через find_package)
# git clone --branch boost-1.83.0 --depth 1 https://github.com/boostorg/boost.git
# cd boost
# ./bootstrap.sh
# ./b2 install --prefix=../boost-install
# cd ..

# === libigl (використовується для геометрії)
git clone --depth 1 https://github.com/libigl/libigl.git || echo "libigl already cloned"

# === libnest2d (Nested shape placement)
git clone https://github.com/Ultimaker/libnest2d.git || echo "libnest2d already cloned"

# === Qhull (геометрія полігонів)
git clone https://github.com/qhull/qhull.git || echo "qhull already cloned"

# === nlopt (оптимізація)
git clone https://github.com/stevengj/nlopt.git || echo "nlopt already cloned"
cd nlopt && cmake -B build && cmake --build build && cd ..

# === cereal (сереалізація)
git clone https://github.com/USCiLab/cereal.git || echo "cereal already cloned"

# === Clipper2 (геометричні операції)
git clone https://github.com/AngusJohnson/Clipper2.git || echo "Clipper2 already cloned"

# === admesh (STL імпорт)
git clone https://github.com/admesh/admesh.git || echo "admesh already cloned"
cd admesh && cmake -B build && cmake --build build && cd ..

# === libexpat (якщо не system)
# wget https://github.com/libexpat/libexpat/releases/download/R_2_6_0/expat-2.6.0.tar.gz

# === додаткові (за потреби):
# - libpng
# - GLEW
# - OpenVDB
# - wxWidgets (GUI, коментовано)
# - imgui (GUI, коментовано)

echo "✅ Завантаження завершено. Перевірте src/deps/"
