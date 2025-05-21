#!/bin/bash

SRC_DIR="$1"
DST_DIR="$2"

mkdir -p "$DST_DIR/libs"

cp -r "$SRC_DIR/src/libslic3r" "$DST_DIR/libs/"
cp -r "$SRC_DIR/src/libslic3r/Geometry" "$DST_DIR/libs/libslic3r_geometry"
cp -r "$SRC_DIR/src/qhull" "$DST_DIR/libs/libqhullcpp"

echo "Sources copied to $DST_DIR"
