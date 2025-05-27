#!/bin/bash
# Встановлення системних бібліотек, потрібних для CLI-слайсингу

sudo apt update
sudo apt install -y \
  build-essential \
  cmake \
  libboost-filesystem-dev \
  libboost-iostreams-dev \
  libboost-system-dev \
  libboost-thread-dev \
  libtbb-dev \
  libeigen3-dev \
  libnlopt-dev \
  libexpat1-dev \
  libpng-dev \
  zlib1g-dev \
  libjpeg-dev \
  libglu1-mesa-dev \
  libfreetype-dev \
  libfontconfig1-dev
