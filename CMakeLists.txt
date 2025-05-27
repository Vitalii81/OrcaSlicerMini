cmake_minimum_required(VERSION 3.13)
project(OrcaSlicer-native)

add_subdirectory(build-utils)
add_subdirectory(admesh)
# add_subdirectory(avrdude)
add_subdirectory(clipper)
add_subdirectory(clipper2)
add_subdirectory(miniz)
add_subdirectory(minilzo)
add_subdirectory(glu-libtess)
add_subdirectory(qhull)
add_subdirectory(Shiny)
add_subdirectory(semver)
add_subdirectory(libigl)
add_subdirectory(hints)
add_subdirectory(mcut)
add_subdirectory(qoi)

# Adding libnest2d project for bin packing...
add_subdirectory(libnest2d)

add_subdirectory(libslic3r)

if (SLIC3R_GUI)
    add_subdirectory(imgui)
    add_subdirectory(imguizmo)
    add_subdirectory(hidapi)
    include_directories(hidapi/include)

    if(WIN32)
        message(STATUS "WXWIN environment set to: $ENV{WXWIN}")
    elseif(UNIX)
        set(wxWidgets_USE_UNICODE ON)
        if(SLIC3R_STATIC)
            set(wxWidgets_USE_STATIC ON)
        else()
            set(wxWidgets_USE_STATIC OFF)
        endif()
    endif()

    if (CMAKE_SYSTEM_NAME STREQUAL "Linux")
        set (wxWidgets_CONFIG_OPTIONS "--toolkit=gtk${SLIC3R_GTK}")
        if (SLIC3R_WX_STABLE)
            find_package(wxWidgets 3.0 REQUIRED COMPONENTS base core adv html gl aui net media webview)
        else ()
            find_package(wxWidgets 3.1 REQUIRED COMPONENTS base core adv html gl aui net media webview)
        endif ()
    else ()
        find_package(wxWidgets 3.1 REQUIRED COMPONENTS html adv gl core base webview aui net media)
    endif ()

    if(UNIX)
        message(STATUS "wx-config path: ${wxWidgets_CONFIG_EXECUTABLE}")
    endif()

    include(${wxWidgets_USE_FILE})

    find_package(JPEG QUIET)
    find_package(TIFF QUIET)

    string(REGEX MATCH "wxpng" WX_PNG_BUILTIN ${wxWidgets_LIBRARIES})
    if (PNG_FOUND AND NOT WX_PNG_BUILTIN)
        list(FILTER wxWidgets_LIBRARIES EXCLUDE REGEX png)
        list(APPEND wxWidgets_LIBRARIES ${PNG_LIBRARIES})
    endif ()

    string(REGEX MATCH "wxtiff" WX_TIFF_BUILTIN ${wxWidgets_LIBRARIES})
    if (TIFF_FOUND AND NOT WX_TIFF_BUILTIN)
        list(FILTER wxWidgets_LIBRARIES EXCLUDE REGEX tiff)
        list(APPEND wxWidgets_LIBRARIES ${TIFF_LIBRARIES})
    endif ()

    string(REGEX MATCH "wxjpeg" WX_JPEG_BUILTIN ${wxWidgets_LIBRARIES})
    if (JPEG_FOUND AND NOT WX_JPEG_BUILTIN)
        list(FILTER wxWidgets_LIBRARIES EXCLUDE REGEX jpeg)
        list(APPEND wxWidgets_LIBRARIES ${JPEG_LIBRARIES})
    endif ()

    string(REGEX MATCH "wxexpat" WX_EXPAT_BUILTIN ${wxWidgets_LIBRARIES})
    if (EXPAT_FOUND AND NOT WX_EXPAT_BUILTIN)
        list(FILTER wxWidgets_LIBRARIES EXCLUDE REGEX expat)
        list(APPEND wxWidgets_LIBRARIES ${EXPAT_LIBRARIES})
    endif ()
    
    # This is an issue in the new wxWidgets cmake build, doesn't deal with librt
    find_library(LIBRT rt)
    if(LIBRT)
        list(APPEND wxWidgets_LIBRARIES ${LIBRT})
    endif()

    # This fixes a OpenGL linking issue on OSX. wxWidgets cmake build includes
    # wrong libs for opengl in the link line and it does not link to it by himself.
    # libslic3r_gui will link to opengl anyway, so lets override wx
    list(FILTER wxWidgets_LIBRARIES EXCLUDE REGEX OpenGL)

#    list(REMOVE_ITEM wxWidgets_LIBRARIES oleacc)
    message(STATUS "wx libs: ${wxWidgets_LIBRARIES}")

    add_subdirectory(slic3r)
endif()

if(ORCA_TOOLS)
    # OrcaSlicer_profile_validator
    add_executable(OrcaSlicer_profile_validator OrcaSlicer_profile_validator.cpp)
    target_link_libraries(OrcaSlicer_profile_validator libslic3r boost_headeronly libcurl OpenSSL::SSL OpenSSL::Crypto)
    target_compile_definitions(OrcaSlicer_profile_validator PRIVATE -DBOOST_ALL_NO_LIB -DBOOST_USE_WINAPI_VERSION=0x602 -DBOOST_SYSTEM_USE_UTF8)
    if(WIN32)
        target_link_libraries(OrcaSlicer_profile_validator bcrypt.lib)
    endif()
endif()

# Create a slic3r executable
# Process mainfests for various platforms.
# configure_file(${CMAKE_CURRENT_SOURCE_DIR}/platform/msw/OrcaSlicer.rc.in ${CMAKE_CURRENT_BINARY_DIR}/OrcaSlicer.rc @ONLY)
# configure_file(${CMAKE_CURRENT_SOURCE_DIR}/platform/msw/OrcaSlicer.manifest.in ${CMAKE_CURRENT_BINARY_DIR}/OrcaSlicer.manifest @ONLY)
# configure_file(${CMAKE_CURRENT_SOURCE_DIR}/platform/osx/Info.plist.in ${CMAKE_CURRENT_BINARY_DIR}/Info.plist @ONLY)
# if (SLIC3R_GUI)
#     add_executable(OrcaSlicer OrcaSlicer.cpp OrcaSlicer.hpp)
# else()

    add_executable(OrcaSlicer OrcaSlicer_cli.cpp OrcaSlicer_cli.hpp)
#endif()

if (NOT WIN32 AND NOT APPLE)
    # Binary name on unix like systems (Linux, Unix)
    set_target_properties(OrcaSlicer PROPERTIES OUTPUT_NAME "orca-slicer")
    set(SLIC3R_APP_CMD "orca-slicer")
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/platform/unix/BuildLinuxImage.sh.in ${CMAKE_CURRENT_BINARY_DIR}/BuildLinuxImage.sh @ONLY)
endif ()

target_link_libraries(OrcaSlicer libslic3r cereal::cereal)

# Boost on Raspberry-Pi does not link to pthreads explicitely.
target_link_libraries(OrcaSlicer ${CMAKE_DL_LIBS} -lstdc++ Threads::Threads pangoft2-1.0)

# Add the Slic3r GUI library, libcurl, OpenGL and GLU libraries.
if (SLIC3R_GUI)
#    target_link_libraries(OrcaSlicer ws2_32 uxtheme setupapi libslic3r_gui ${wxWidgets_LIBRARIES})
target_link_libraries(OrcaSlicer libslic3r_gui)
target_link_libraries(OrcaSlicer -ldl)
endif ()

# Link the resources dir to where Slic3r GUI expects it
set(output_dlls_Release "")
set(output_dlls_Debug "")
set(output_dlls_RelWithDebInfo "")

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

