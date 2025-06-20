
#ifndef ORCA_SLICER_CLI_H
#define ORCA_SLICER_CLI_H

#include "libslic3r/Print.hpp"
#include "libslic3r/TriangleMesh.hpp"

enum TypeFile { TypeSTL, TypeSTEP, Type3MF, TypeOBJ, TypeUnknown };

/**
 * Structure representing a 2D point with float coordinates
 */
struct Point_ {
    float x, y;  // X and Y coordinates
};

/**
 * Structure representing a contour/polygon composed of points
 */
struct Contour {
    std::vector<Point_> points;  // Collection of points forming the contour
    bool is_outer = true;  // true — outer perimeter, false — hole (inner)
};

/**
 * Structure representing data for a single layer
 */
struct LayerData {
    uint32_t __size;             // Size of layer data in bytes (for serialization)
    uint32_t layer_id;           // Identifier of the layer
    double z_height;             // Z-coordinate/height of the layer
    std::vector<Contour> perimeters;  // Collection of contours (outer perimeter and holes)
};

/**
 * Structure containing all slicing data for a model
 */
struct SlicingData {
    std::vector<LayerData> layers;  // Collection of all layers
};

std::unique_ptr<Slic3r::Print> slicing_stl(const char* path, float layer_height);


#endif