cmake_minimum_required(VERSION 3.13)
project(libslic3r)

string(TIMESTAMP COMPILE_TIME %Y%m%d-%H%M%S)
set(SLIC3R_BUILD_TIME ${COMPILE_TIME})

if(NOT DEFINED ORCA_CHECK_GCODE_PLACEHOLDERS)
    set(ORCA_CHECK_GCODE_PLACEHOLDERS "0")
endif()

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/libslic3r_version.h.in ${CMAKE_CURRENT_BINARY_DIR}/libslic3r_version.h @ONLY)

#set(OpenVDBUtils_SOURCES "")
#if (TARGET OpenVDB::openvdb)
#    set(OpenVDBUtils_SOURCES OpenVDBUtils.cpp OpenVDBUtils.hpp)
#endif()

set(lisbslic3r_sources
        ArcFitter.cpp     # Аппроксимация дуг окружностей
        ArcFitter.hpp     # Заголовок для ArcFitter
        pchheader.cpp     # Прекомпіляція заголовків
        pchheader.hpp     # Заголовок для пре-компіляції
        AABBTreeIndirect.hpp    # Просторова структура AABB (індексована)
        AABBTreeLines.hpp       # AABB для ліній
        AABBMesh.hpp            # AABB для трикутної сітки
        AABBMesh.cpp
        Algorithm/LineSplit.hpp         # Алгоритм поділу ліній
        Algorithm/LineSplit.cpp
        Algorithm/RegionExpansion.hpp   # Розширення регіону (області друку)
        Algorithm/RegionExpansion.cpp
        AnyPtr.hpp              # Універсальний вказівник (type-erased)
        BoundingBox.cpp         # Визначення прямокутної області об'єкта
        BoundingBox.hpp
        BuildVolume.cpp         # Обробка об'єму будови
        BuildVolume.hpp
        Circle.cpp              # Геометрія кола
        Circle.hpp
        clipper.cpp             # Boolean-операції над полігонами
        clipper.hpp
        ClipperUtils.cpp        # Утиліти для Clipper
        ClipperUtils.hpp
        Clipper2Utils.cpp       # Утиліти для Clipper2
        Clipper2Utils.hpp
        ClipperZUtils.hpp       # 3D Clipper утиліти
        Color.cpp               # Кольори об'єктів
        Color.hpp
        Config.cpp              # Конфігураційні параметри
        Config.hpp
        EdgeGrid.cpp            # Грані у вигляді сітки
        EdgeGrid.hpp

        enum_bitmask.hpp        # Робота з enum'ами як бітовими масками
        ExPolygon.cpp           # Розширений полігон (з отворами)
        ExPolygon.hpp
        ExPolygonSerialize.hpp  # Серіалізація ExPolygon
        ExPolygonsIndex.cpp     # Індексування полігонів
        ExPolygonsIndex.hpp

        FileParserError.hpp     # Обробка помилок парсингу файлів
        format.hpp              # Форматування рядків
        Format/3mf.cpp          # Парсер 3MF
        Format/3mf.hpp
        Format/bbs_3mf.cpp      # Парсер кастомного Bambu 3MF
        Format/bbs_3mf.hpp
        Format/OBJ.cpp          # Парсер OBJ-файлів
        Format/OBJ.hpp
        Format/objparser.cpp
        Format/objparser.hpp
        Format/STEP.cpp         # Парсер STEP
        Format/STEP.hpp
        Format/STL.cpp          # Парсер STL
        Format/STL.hpp
        Format/ZipperArchiveImport.hpp # Імпорт ZIP-архівів
        Format/ZipperArchiveImport.cpp

        GCode/ThumbnailData.cpp # Генерація прев’ю (thumbnail)
        GCode/ThumbnailData.hpp
        Geometry.cpp            # Геометричні операції
        Geometry.hpp
        Geometry/Bicubic.hpp    # Бікубічна інтерполяція
        Geometry/Circle.cpp
        Geometry/Circle.hpp
        Geometry/ConvexHull.cpp # Опукла оболонка
        Geometry/ConvexHull.hpp
        Geometry/Curves.hpp
        Int128.hpp              # Робота з 128-бітними цілими
        KDTreeIndirect.hpp      # KD-дерево для швидкого пошуку
        Layer.cpp               # Реалізація шару друку
        Layer.hpp
        LayerRegion.cpp         # Область в шарі
        libslic3r.h             # Основний заголовок
        libslic3r.cpp           # Основна точка входу
        Line.cpp
        Line.hpp
        BlacklistedLibraryCheck.cpp # Перевірка проблемних бібліотек
        BlacklistedLibraryCheck.hpp
        LocalesUtils.cpp        # Мовна локалізація
        LocalesUtils.hpp
        CutUtils.cpp            # Нарізка моделей
        CutUtils.hpp
        Model.cpp               # Представлення моделі
        Model.hpp
        NormalUtils.cpp         # Робота з нормалями
        NormalUtils.hpp
        ObjColorUtils.hpp       # Обробка кольорів в OBJ
        Orient.hpp              # Орієнтація моделей
        Orient.cpp
        MultiPoint.cpp
        MultiPoint.hpp
        MutablePriorityQueue.hpp # Черга з пріоритетами
        ObjectID.cpp            # Унікальні ID об'єктів
        ObjectID.hpp
        ParameterUtils.cpp      # Утиліти для параметрів
        ParameterUtils.hpp
        PerimeterGenerator.cpp  # Генерація периметрів
        PerimeterGenerator.hpp

        Point.cpp               # Точка
        Point.hpp
        Polygon.cpp             # Полігони
        Polygon.hpp
        MutablePolygon.cpp      # Змінні полігони
        MutablePolygon.hpp
        PolygonTrimmer.cpp      # Обрізка полігонів
        PolygonTrimmer.hpp
        Polyline.cpp            # Ламані лінії
        Polyline.hpp

        PrincipalComponents2D.hpp # Головні компоненти для 2D
        PrincipalComponents2D.cpp
        AppConfig.cpp           # Глобальна конфігурація
        AppConfig.hpp
        Print.cpp               # Головний клас друку
        Print.hpp
        PrintApply.cpp          # Застосування параметрів
        PrintBase.cpp
        PrintBase.hpp
        PrintConfig.cpp         # Конфігурація принта
        PrintConfig.hpp
        PrintObject.cpp         # Об'єкт для друку
        PrintObjectSlice.cpp    # Нарізка об'єкта
        PrintRegion.cpp         # Регіони в об'єкті
        PNGReadWrite.hpp        # Читання/запис PNG
        PNGReadWrite.cpp
        QuadricEdgeCollapse.cpp # Спрощення сітки
        QuadricEdgeCollapse.hpp
        ShortEdgeCollapse.cpp   # Колапс коротких ребер
        ShortEdgeCollapse.hpp
        Slicing.cpp             # Основний slicing
        Slicing.hpp
        SlicesToTriangleMesh.hpp
        SlicesToTriangleMesh.cpp
        SlicingAdaptive.cpp     # Адаптивне slicing
        SlicingAdaptive.hpp
        MinimumSpanningTree.hpp # Алгоритм MST
        MinimumSpanningTree.cpp
        Surface.cpp             # Поверхні друку
        Surface.hpp
        SurfaceCollection.cpp
        SurfaceCollection.hpp
        SurfaceMesh.hpp         # Поверхнева сітка
        Technologies.hpp        # Технології (типи друку)
        Tesselate.cpp           # Теселяція геометрії
        Tesselate.hpp
        TextConfiguration.hpp   # Конфігурація тексту
        TriangleMesh.cpp        # Меш (сітка)
        TriangleMesh.hpp
        TriangleMeshSlicer.cpp  # Нарізка сітки
        TriangleMeshSlicer.hpp
        MeshSplitImpl.hpp       # Поділ мешу
        TriangulateWall.hpp     # Триангуляція стінок
        TriangulateWall.cpp
        utils.cpp               # Загальні утиліти
        Utils.hpp
        Time.cpp
        Time.hpp
        TriangleSelector.cpp    # Вибір трикутників
        TriangleSelector.hpp
        TriangleSetSampling.cpp # Семплювання трикутників
        TriangleSetSampling.hpp
        MTUtils.hpp
        Zipper.hpp              # Робота з ZIP-архівами
        Zipper.cpp
        MinAreaBoundingBox.hpp  # Мінімальна обгортка (BBox)
        MinAreaBoundingBox.cpp
        miniz_extension.hpp     # Підтримка архівації
        miniz_extension.cpp
        MarchingSquares.hpp     # Алгоритм Marching Squares

        Execution/ExecutionTBB.hpp      # Паралельне виконання
        Optimize/Optimizer.hpp          # Загальна оптимізація
        Optimize/NLoptOptimizer.hpp     # Оптимізатор NLopt
        Optimize/BruteforceOptimizer.hpp # Оптимізація перебором
        Shape/TextShape.cpp             # Текстові мітки
        GCode/Thumbnails.cpp            # Зображення в GCode
)


add_library(libslic3r STATIC ${lisbslic3r_sources}
        "${CMAKE_CURRENT_BINARY_DIR}/libslic3r_version.h"
#        ${OpenVDBUtils_SOURCES}
)
source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${lisbslic3r_sources})

if (SLIC3R_STATIC)
    set(CGAL_Boost_USE_STATIC_LIBS ON CACHE BOOL "" FORCE)
endif ()
set(CGAL_DO_NOT_WARN_ABOUT_CMAKE_BUILD_TYPE ON CACHE BOOL "" FORCE)

cmake_policy(PUSH)
cmake_policy(SET CMP0011 NEW)
find_package(CGAL REQUIRED)
#find_package(OpenCV REQUIRED core)
find_package(JPEG REQUIRED)
find_package(PNG REQUIRED)
cmake_policy(POP)

add_library(libslic3r_cgal STATIC
       # CutSurface.hpp CutSurface.cpp
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
        libnest2d
        admesh
        cereal::cereal
        libigl
        miniz
        boost_libs
        clipper
        ${EXPAT_LIBRARIES}
        glu-libtess
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
        #qoi
        #OpenCV::OpenCV
        noise
        freetype
        fontconfig
)

#if (TARGET OpenVDB::openvdb)
#    target_link_libraries(libslic3r OpenVDB::openvdb)
#endif()

if(SLIC3R_PROFILE)
    target_link_libraries(libslic3r Shiny)
endif()