cmake_minimum_required(VERSION 3.13)
project(OrcaSlicer-native)

add_subdirectory(build-utils)
add_subdirectory(admesh)
# add_subdirectory(avrdude)
add_subdirectory(clipper)
add_subdirectory(clipper2)
add_subdirectory(miniz)
add_subdirectory(minilzo)
#add_subdirectory(glu-libtess)
add_subdirectory(qhull)
add_subdirectory(semver)
add_subdirectory(libigl)
add_subdirectory(hints)
add_subdirectory(mcut)
#add_subdirectory(qoi)

# Adding libnest2d project for bin packing...
#add_subdirectory(libnest2d)

add_subdirectory(libslic3r)

add_executable(OrcaSlicer OrcaSlicer_cli.cpp OrcaSlicer_cli.hpp)

    # Binary name on unix like systems (Linux, Unix)
    set_target_properties(OrcaSlicer PROPERTIES OUTPUT_NAME "orca-slicer")
    set(SLIC3R_APP_CMD "orca-slicer")
#    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/platform/unix/BuildLinuxImage.sh.in ${CMAKE_CURRENT_BINARY_DIR}/BuildLinuxImage.sh @ONLY)

target_link_libraries(OrcaSlicer libslic3r cereal::cereal)

# Boost on Raspberry-Pi does not link to pthreads explicitely.
target_link_libraries(OrcaSlicer ${CMAKE_DL_LIBS} -lstdc++ Threads::Threads pangoft2-1.0)

# Slic3r binary install target. Default build type is release in case no CMAKE_BUILD_TYPE is provided.
if( ("${CMAKE_BUILD_TYPE}" STREQUAL "Release") OR ("${CMAKE_BUILD_TYPE}" STREQUAL "") )
    set (build_type "Release")
    set(CMAKE_BUILD_POSTFIX "")
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "RelWithDebInfo")
    set (build_type "RelWithDebInfo")
    set(CMAKE_BUILD_POSTFIX "")
else()
    set (build_type "Debug")
    set(CMAKE_BUILD_POSTFIX "d")
endif()
message(STATUS "libslic3r-CMAKE_BUILD_TYPE: ${build_type}")
message(STATUS "CMAKE_CURRENT_BINARY_DIR: ${CMAKE_CURRENT_BINARY_DIR}")

install(TARGETS OrcaSlicer RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" BUNDLE DESTINATION ${CMAKE_INSTALL_BINDIR})

