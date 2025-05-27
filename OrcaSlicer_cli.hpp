
#ifndef ORCA_SLICER_CLI_H
#define ORCA_SLICER_CLI_H

#include "libslic3r/Print.hpp"
#include "libslic3r/TriangleMesh.hpp"

enum TypeFile
{
    TypeSTL,
    TypeStep
};

std::unique_ptr<Slic3r::Print> slicing_stl(const char* path, float layer_height, TypeFile type = TypeSTL);

#endif