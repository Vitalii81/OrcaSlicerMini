cmake_minimum_required(VERSION 3.13)
project(libslic3r)

string(TIMESTAMP COMPILE_TIME %Y%m%d-%H%M%S)
set(SLIC3R_BUILD_TIME ${COMPILE_TIME})

if(NOT DEFINED ORCA_CHECK_GCODE_PLACEHOLDERS)
    set(ORCA_CHECK_GCODE_PLACEHOLDERS "0")
endif()

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/libslic3r_version.h.in ${CMAKE_CURRENT_BINARY_DIR}/libslic3r_version.h @ONLY)

set(lisbslic3r_sources
        ArcFitter.cpp     # Arc approximation
        ArcFitter.hpp     # Header for ArcFitter
        pchheader.cpp     # Precompiled headers
        pchheader.hpp     # Header for precompiled headers
        AABBTreeIndirect.hpp    # Spatial structure AABB (indexed)
        AABBTreeLines.hpp       # AABB for lines
        AABBMesh.hpp            # AABB for triangle mesh
        AABBMesh.cpp
        Algorithm/LineSplit.hpp         # Line splitting algorithm
        Algorithm/LineSplit.cpp
        Algorithm/RegionExpansion.hpp   # Region (print area) expansion
        Algorithm/RegionExpansion.cpp
        AnyPtr.hpp              # Universal pointer (type-erased)
        BoundingBox.cpp         # Bounding box definition
        BoundingBox.hpp
        BuildVolume.cpp         # Build volume handling
        BuildVolume.hpp
        Circle.cpp              # Circle geometry
        Circle.hpp
        clipper.cpp             # Boolean operations on polygons
        clipper.hpp
        ClipperUtils.cpp        # Utilities for Clipper
        ClipperUtils.hpp
        Clipper2Utils.cpp       # Utilities for Clipper2
        Clipper2Utils.hpp
        ClipperZUtils.hpp       # 3D Clipper utilities
        Color.cpp               # Object colors
        Color.hpp
        Config.cpp              # Configuration parameters
        Config.hpp
        EdgeGrid.cpp            # Edges as a grid
        EdgeGrid.hpp

        enum_bitmask.hpp        # Working with enums as bitmasks
        ExPolygon.cpp           # Extended polygon (with holes)
        ExPolygon.hpp
        ExPolygonSerialize.hpp  # ExPolygon serialization
        ExPolygonsIndex.cpp     # Polygon indexing
        ExPolygonsIndex.hpp

        FileParserError.hpp     # File parsing error handling
        format.hpp              # String formatting
        Format/3mf.cpp          # 3MF parser
        Format/3mf.hpp
        Format/bbs_3mf.cpp      # Custom Bambu 3MF parser
        Format/bbs_3mf.hpp
        Format/OBJ.cpp          # OBJ file parser
        Format/OBJ.hpp
        Format/objparser.cpp
        Format/objparser.hpp
        Format/STEP.cpp         # STEP parser
        Format/STEP.hpp
        Format/STL.cpp          # STL parser
        Format/STL.hpp
        Format/ZipperArchiveImport.hpp # ZIP archive import
        Format/ZipperArchiveImport.cpp

        GCode/ThumbnailData.cpp # Preview thumbnail generation
        GCode/ThumbnailData.hpp
        Geometry.cpp            # Geometric operations
        Geometry.hpp
        Geometry/Bicubic.hpp    # Bicubic interpolation
        Geometry/Circle.cpp
        Geometry/Circle.hpp
        Geometry/ConvexHull.cpp # Convex hull
        Geometry/ConvexHull.hpp
        Geometry/Curves.hpp
        Int128.hpp              # 128-bit integer handling
        KDTreeIndirect.hpp      # KD-tree for fast lookup
        Layer.cpp               # Print layer implementation
        Layer.hpp
        LayerRegion.cpp         # Region within a layer
        libslic3r.h             # Main header
        libslic3r.cpp           # Main entry point
        Line.cpp
        Line.hpp
        BlacklistedLibraryCheck.cpp # Problematic libraries check
        BlacklistedLibraryCheck.hpp
        LocalesUtils.cpp        # Language localization
        LocalesUtils.hpp
        CutUtils.cpp            # Model cutting
        CutUtils.hpp
        Model.cpp               # Model representation
        Model.hpp
        NormalUtils.cpp         # Normal vector handling
        NormalUtils.hpp
        ObjColorUtils.hpp       # OBJ color handling
        Orient.hpp              # Model orientation
        Orient.cpp
        MultiPoint.cpp
        MultiPoint.hpp
        MutablePriorityQueue.hpp # Priority queue
        ObjectID.cpp            # Unique object IDs
        ObjectID.hpp
        ParameterUtils.cpp      # Parameter utilities
        ParameterUtils.hpp
        PerimeterGenerator.cpp  # Perimeter generation
        PerimeterGenerator.hpp

        Point.cpp               # Point
        Point.hpp
        Polygon.cpp             # Polygons
        Polygon.hpp
        MutablePolygon.cpp      # Mutable polygons
        MutablePolygon.hpp
        PolygonTrimmer.cpp      # Polygon trimming
        PolygonTrimmer.hpp
        Polyline.cpp            # Polylines
        Polyline.hpp

        PrincipalComponents2D.hpp # Principal components for 2D
        PrincipalComponents2D.cpp
        AppConfig.cpp           # Global configuration
        AppConfig.hpp
        Print.cpp               # Main print class
        Print.hpp
        PrintApply.cpp          # Parameter application
        PrintBase.cpp
        PrintBase.hpp
        PrintConfig.cpp         # Print configuration
        PrintConfig.hpp
        PrintObject.cpp         # Printable object
        PrintObjectSlice.cpp    # Object slicing
        PrintRegion.cpp         # Regions in the object
        PNGReadWrite.hpp        # PNG read/write
        PNGReadWrite.cpp
        QuadricEdgeCollapse.cpp # Mesh simplification
        QuadricEdgeCollapse.hpp
        ShortEdgeCollapse.cpp   # Short edge collapse
        ShortEdgeCollapse.hpp
        Slicing.cpp             # Main slicing logic
        Slicing.hpp
        SlicesToTriangleMesh.hpp
        SlicesToTriangleMesh.cpp
        SlicingAdaptive.cpp     # Adaptive slicing
        SlicingAdaptive.hpp
        MinimumSpanningTree.hpp # MST algorithm
        MinimumSpanningTree.cpp
        Surface.cpp             # Print surfaces
        Surface.hpp
        SurfaceCollection.cpp
        SurfaceCollection.hpp
        SurfaceMesh.hpp         # Surface mesh
        Technologies.hpp        # Print technologies
        #Tesselate.cpp           # Geometry tessellation
        #Tesselate.hpp
        TextConfiguration.hpp   # Text configuration
        TriangleMesh.cpp        # Mesh
        TriangleMesh.hpp
        TriangleMeshSlicer.cpp  # Mesh slicing
        TriangleMeshSlicer.hpp
        MeshSplitImpl.hpp       # Mesh splitting
        TriangulateWall.hpp     # Wall triangulation
        TriangulateWall.cpp
        utils.cpp               # General utilities
        Utils.hpp
        Time.cpp
        Time.hpp
        TriangleSelector.cpp    # Triangle selection
        TriangleSelector.hpp
        TriangleSetSampling.cpp # Triangle sampling
        TriangleSetSampling.hpp
        MTUtils.hpp
        Zipper.hpp              # ZIP archive handling
        Zipper.cpp

        miniz_extension.hpp     # Archiving support
        miniz_extension.cpp
        MarchingSquares.hpp     # Marching Squares algorithm
        Execution/Execution.hpp         # Execution interface
        Execution/ExecutionSeq.hpp      # Sequential execution
        Execution/ExecutionTBB.hpp      # Parallel execution
        GCode/Thumbnails.cpp            # Thumbnails in GCode
)

add_library(libslic3r STATIC ${lisbslic3r_sources}
        "${CMAKE_CURRENT_BINARY_DIR}/libslic3r_version.h"
)
source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${lisbslic3r_sources})

if (SLIC3R_STATIC)
    set(CGAL_Boost_USE_STATIC_LIBS ON CACHE BOOL "" FORCE)
endif ()
set(CGAL_DO_NOT_WARN_ABOUT_CMAKE_BUILD_TYPE ON CACHE BOOL "" FORCE)

cmake_policy(PUSH)
cmake_policy(SET CMP0011 NEW)
find_package(CGAL REQUIRED)
find_package(JPEG REQUIRED)
find_package(PNG REQUIRED)
cmake_policy(POP)

add_library(libslic3r_cgal STATIC
        IntersectionPoints.hpp IntersectionPoints.cpp
        MeshBoolean.hpp MeshBoolean.cpp
        TryCatchSignal.hpp TryCatchSignal.cpp
        Triangulation.hpp Triangulation.cpp
)
target_include_directories(libslic3r_cgal PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

get_target_property(_cgal_tgt CGAL::CGAL ALIASED_TARGET)
if (NOT TARGET ${_cgal_tgt})
    set (_cgal_tgt CGAL::CGAL)
endif ()
get_target_property(_opts ${_cgal_tgt} INTERFACE_COMPILE_OPTIONS)
if (_opts)
    set(_opts_bad "${_opts}")
    set(_opts_good "${_opts}")
    list(FILTER _opts_bad INCLUDE REGEX frounding-math)
    list(FILTER _opts_good EXCLUDE REGEX frounding-math)
    set_target_properties(${_cgal_tgt} PROPERTIES INTERFACE_COMPILE_OPTIONS "${_opts_good}")
    target_compile_options(libslic3r_cgal PRIVATE "${_opts_bad}")
endif()

target_link_libraries(libslic3r_cgal PRIVATE ${_cgal_tgt} libigl mcut boost_libs)

encoding_check(libslic3r)

target_compile_definitions(libslic3r PUBLIC -DUSE_TBB -DTBB_USE_CAPTURED_EXCEPTION=0)
target_include_directories(libslic3r PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
target_include_directories(libslic3r PUBLIC ${EXPAT_INCLUDE_DIRS})

set(OpenCASCADE_DIR "${CMAKE_PREFIX_PATH}/lib/cmake/occt")
find_package(OpenCASCADE REQUIRED)
target_include_directories(libslic3r PUBLIC ${OpenCASCADE_INCLUDE_DIR})

set(OCCT_LIBS
        TKXDESTEP
        TKSTEP
        TKSTEP209
        TKSTEPAttr
        TKSTEPBase
        TKXCAF
        TKXSBase
        TKVCAF
        TKCAF
        TKLCAF
        TKCDF
        TKV3d
        TKService
        TKMesh
        TKBO
        TKPrim
        TKHLR
        TKShHealing
        TKTopAlgo
        TKGeomAlgo
        TKBRep
        TKGeomBase
        TKG3d
        TKG2d
        TKMath
        TKernel
)

target_link_libraries(libslic3r

        admesh
        cereal::cereal
        libigl
        miniz
        boost_libs
        clipper
        ${EXPAT_LIBRARIES}
        #glu-libtess
        qhull
        semver
        TBB::tbb
        TBB::tbbmalloc
        libslic3r_cgal
        ${CMAKE_DL_LIBS}
        PNG::PNG
        ZLIB::ZLIB
        ${OCCT_LIBS}
        Clipper2
        mcut
        JPEG::JPEG
        noise
        freetype
        fontconfig
)
