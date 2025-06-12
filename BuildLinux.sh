#!/bin/bash

export ROOT=$(dirname $(readlink -f ${0}))

sudo apt install m4 -y
sudo apt install texinfo -y
sudo apt install autoconf automake libtool -y

set -e # exit on first error


function usage() {
    echo "Usage: ./BuildLinux.sh [-1][-b][-c][-d][-i][-r][-s][-u] [-j N]"
    echo "   -1: limit builds to 1 core (where possible)"
    echo "   -j N: limit builds to N cores (where possible)"
    echo "   -b: build in debug mode"
    echo "   -c: force a clean build"
    echo "   -d: build deps (optional)"
    echo "   -h: this help output"
    echo "   -s: build orca-slicer (optional)"
    echo "   -u: update and build dependencies (optional and need sudo)"
    echo "For a first use, you want to 'sudo ./BuildLinux.sh -u'"
    echo "   and then './BuildLinux.sh -dsi'"
}

unset name
while getopts ":1j:bcdghirsu" opt; do
  case ${opt} in
    1 )
        export CMAKE_BUILD_PARALLEL_LEVEL=1
        ;;
    j )
        export CMAKE_BUILD_PARALLEL_LEVEL=$OPTARG
        ;;
    b )
        BUILD_DEBUG="1"
        ;;
    c )
        CLEAN_BUILD=1
        ;;
    d )
        BUILD_DEPS="1"
        ;;
    h ) usage
        exit 0
        ;;
    s )
        BUILD_ORCA="1"
        ;;
    u )
        UPDATE_LIB="1"
        ;;
  esac
done

if [ ${OPTIND} -eq 1 ]
then
    usage
    exit 0
fi

DISTRIBUTION="debian"
source ./linux.d/${DISTRIBUTION}

if [[ -n "${BUILD_DEPS}" ]]
then
    echo "Configuring dependencies..."
    BUILD_ARGS="-DDEP_WX_GTK3=OFF"
    if [[ -n "${CLEAN_BUILD}" ]]
    then
        rm -fr deps/build
    fi
    if [ ! -d "deps/build" ]
    then
        mkdir deps/build
    fi
    if [[ -n "${BUILD_DEBUG}" ]]
    then
        # have to build deps with debug & release or the cmake won't find everything it needs
        if [ ! -d "deps/build/release" ]
        then
            mkdir deps/build/release
        fi
        cmake -S deps -B deps/build/release -G Ninja -DDESTDIR="${PWD}/deps/build/destdir" -DDEP_DOWNLOAD_DIR="${PWD}/deps/DL_CACHE" ${BUILD_ARGS}
        cmake --build deps/build/release
        BUILD_ARGS="${BUILD_ARGS} -DCMAKE_BUILD_TYPE=Debug"
    fi

    echo "cmake -S deps -B deps/build -G Ninja ${BUILD_ARGS}"
    cmake -S deps -B deps/build -G Ninja ${BUILD_ARGS} 2>&1 | tee deps_config.log
    cmake --build deps/build --verbose 2>&1 | tee deps_build.log
fi


if [[ -n "${BUILD_ORCA}" ]]
then
    echo "Configuring OrcaSlicer..."
    if [[ -n "${CLEAN_BUILD}" ]]
    then
        rm -fr build
    fi
    BUILD_ARGS=""

    if [[ -n "${BUILD_DEBUG}" ]]
    then
        BUILD_ARGS="${BUILD_ARGS} -DCMAKE_BUILD_TYPE=Debug -DBBL_INTERNAL_TESTING=1"
    else
        BUILD_ARGS="${BUILD_ARGS} -DBBL_RELEASE_TO_PUBLIC=1 -DBBL_INTERNAL_TESTING=0"
    fi
    echo -e "cmake -S . -B build -G Ninja -DCMAKE_PREFIX_PATH="${PWD}/deps/build/destdir/usr/local" -DSLIC3R_STATIC=1 -DORCA_TOOLS=OFF ${BUILD_ARGS}"
    cmake -S . -B build -G Ninja \
        -DCMAKE_PREFIX_PATH="${PWD}/deps/build/destdir/usr/local" \
        -DSLIC3R_STATIC=1 \
        -DORCA_TOOLS=OFF \
        ${BUILD_ARGS}
    echo "done"
    echo "Building OrcaSlicer ..."
    cmake --build build --target OrcaSlicer
    echo "done"
fi
