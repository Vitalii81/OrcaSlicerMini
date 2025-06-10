cmake_minimum_required(VERSION 3.13)

project(OrcaSlicer)

include(GNUInstallDirs)
include(CMakeDependentOption)

message(STATUS "Build type selected: ${CMAKE_BUILD_TYPE}")

set(SLIC3R_STATIC_INITIAL 1)

option(SLIC3R_STATIC 			"Compile OrcaSlicer with static libraries (Boost, TBB, glew)" ${SLIC3R_STATIC_INITIAL})
option(SLIC3R_FHS               "Assume OrcaSlicer is to be installed in a FHS directory structure" 0)
option(SLIC3R_PCH               "Use precompiled headers" 1)
option(SLIC3R_ASAN              "Enable ASan on Clang and GCC" 0)

set(IS_CROSS_COMPILE FALSE)

option(ORCA_TOOLS       "Build Orca tools" OFF)

# Print out the SLIC3R_* cache options
get_cmake_property(_cache_vars CACHE_VARIABLES)
list (SORT _cache_vars)
foreach (_cache_var ${_cache_vars})
    if("${_cache_var}" MATCHES "^SLIC3R_")
        message(STATUS "${_cache_var}: ${${_cache_var}}")
    endif ()
endforeach()

# ARMs (Raspberry PI) use an unsigned char by default. Let's make it consistent for OrcaSlicer on all platforms.
add_compile_options(-fsigned-char)

# Display and check CMAKE_PREFIX_PATH
message(STATUS "SLIC3R_STATIC: ${SLIC3R_STATIC}")
if (NOT "${CMAKE_PREFIX_PATH}" STREQUAL "")
    message(STATUS "CMAKE_PREFIX_PATH: ${CMAKE_PREFIX_PATH} (from cache or command line)")
    set(PREFIX_PATH_CHECK ${CMAKE_PREFIX_PATH})
elseif (NOT "$ENV{CMAKE_PREFIX_PATH}" STREQUAL "")
    message(STATUS "CMAKE_PREFIX_PATH: $ENV{CMAKE_PREFIX_PATH} (from environment)")
    set(PREFIX_PATH_CHECK $ENV{CMAKE_PREFIX_PATH})
else ()
    message(STATUS "CMAKE_PREFIX_PATH: (default)")
endif ()

foreach (DIR ${PREFIX_PATH_CHECK})
    if (NOT EXISTS "${DIR}")
        message(WARNING "CMAKE_PREFIX_PATH element doesn't exist: ${DIR}")
    endif ()
endforeach ()

# Add our own cmake module path.
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/modules/)
message(STATUS "PROJECT_SOURCE_DIR: ${PROJECT_SOURCE_DIR}")
message(STATUS "CMAKE_MODULE_PATH: ${CMAKE_MODULE_PATH}")

enable_testing ()

# Enable C++17 language standard.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add DEBUG flags to debug builds.
add_compile_definitions("$<IF:$<CONFIG:Debug>,DEBUG,NDEBUG>")

if (CMAKE_SYSTEM_NAME STREQUAL "Linux")
    find_package(PkgConfig REQUIRED)

    if (CMAKE_VERSION VERSION_LESS "3.1")
        # Workaround for an old CMake, which does not understand CMAKE_CXX_STANDARD.
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
    endif()

    # Boost on Raspberry-Pi does not link to pthreads.
    set(THREADS_PREFER_PTHREAD_FLAG ON)
    find_package(Threads REQUIRED)

endif()

if (CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUXX)
    # Adding -fext-numeric-literals to enable GCC extensions on definitions of quad float literals, which are required by Boost.
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fext-numeric-literals" )
endif()

# Where all the bundled libraries reside?
set(LIBDIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(LIBDIR_BIN ${CMAKE_CURRENT_BINARY_DIR}/src)
message(STATUS "LIBDIR: ${LIBDIR}")
message(STATUS "LIBDIR_BIN: ${LIBDIR_BIN}")

# For the bundled boost libraries (boost::nowide)
include_directories(${LIBDIR})

# For ligigl
include_directories(${LIBDIR}/libigl)

# Disable optimization for RelWithDebInfo
if(CMAKE_C_FLAGS_RELWITHDEBINFO MATCHES "/O2")
    string(REGEX REPLACE "/O2" "/Od" CMAKE_C_FLAGS_RELWITHDEBINFO "${CMAKE_C_FLAGS_RELWITHDEBINFO}")
endif()

if(CMAKE_CXX_FLAGS_RELWITHDEBINFO MATCHES "/O2")
    string(REGEX REPLACE "/O2" "/Od" CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO}")
endif()

if("${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang")
    if(CMAKE_C_FLAGS_RELWITHDEBINFO MATCHES "-O2")
        string(REGEX REPLACE "-O2" "-O0" CMAKE_C_FLAGS_RELWITHDEBINFO "${CMAKE_C_FLAGS_RELWITHDEBINFO}")
    endif()

    if(CMAKE_CXX_FLAGS_RELWITHDEBINFO MATCHES "-O2")
        string(REGEX REPLACE "-O2" "-O0" CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO}")
    endif()
endif()


list(REMOVE_DUPLICATES CMAKE_C_FLAGS_RELWITHDEBINFO)
list(REMOVE_DUPLICATES CMAKE_CXX_FLAGS_RELWITHDEBINFO)

# Find and configure boost
if(SLIC3R_STATIC)
    # Use static boost libraries.
    set(Boost_USE_STATIC_LIBS ON)
    # Use boost libraries linked statically to the C++ runtime.
    # set(Boost_USE_STATIC_RUNTIME ON)
else()
    add_definitions(-DBOOST_LOG_DYN_LINK)
endif()

#set(Boost_DEBUG ON)
find_package(Boost 1.83.0 REQUIRED COMPONENTS system filesystem thread log log_setup locale regex chrono atomic date_time iostreams program_options nowide)
add_library(boost_libs INTERFACE)
add_library(boost_headeronly INTERFACE)
if(NOT SLIC3R_STATIC)
    target_compile_definitions(boost_headeronly INTERFACE BOOST_LOG_DYN_LINK)
endif()
target_include_directories(boost_headeronly INTERFACE ${Boost_INCLUDE_DIRS})
target_link_libraries(boost_libs INTERFACE boost_headeronly ${Boost_LIBRARIES})

# Find and configure intel-tbb
if(SLIC3R_STATIC)
    set(TBB_STATIC 1)
endif()
set(TBB_DEBUG 1)
set(CMAKE_MAP_IMPORTED_CONFIG_RELWITHDEBINFO RelWithDebInfo Release "")
find_package(TBB REQUIRED)

#find_package(CURL REQUIRED)
#
#add_library(libcurl INTERFACE)
#target_link_libraries(libcurl INTERFACE CURL::libcurl)
#
#find_package(ZLIB REQUIRED)
#target_link_libraries(libcurl INTERFACE ZLIB::ZLIB)

#if (SLIC3R_STATIC AND NOT SLIC3R_STATIC_EXCLUDE_CURL)
#        # libcurl is always linked dynamically to the system libcurl on OSX.
#        # On other systems, libcurl is linked statically if SLIC3R_STATIC is set.
#        target_compile_definitions(libcurl INTERFACE CURL_STATICLIB)
#
#    if (CMAKE_SYSTEM_NAME STREQUAL "Linux")
#        # As of now, our build system produces a statically linked libcurl,
#        # which links the OpenSSL library dynamically.
#        #find_package(OpenSSL REQUIRED)
#        message("OpenSSL include dir: ${OPENSSL_INCLUDE_DIR}")
#        message("OpenSSL libraries: ${OPENSSL_LIBRARIES}")
#
#    endif()
#endif()

## OPTIONAL packages

# Find eigen3 or use bundled version
if (NOT SLIC3R_STATIC)
    find_package(Eigen3 3.3)
endif ()
if (NOT EIGEN3_FOUND)
    set(EIGEN3_FOUND 1)
    set(EIGEN3_INCLUDE_DIR ${LIBDIR}/eigen/)
endif ()
include_directories(BEFORE SYSTEM ${EIGEN3_INCLUDE_DIR})

# Find expat or use bundled version
# Always use the system libexpat on Linux.

find_package(EXPAT)

if (NOT EXPAT_FOUND)
    add_library(expat STATIC
        ${LIBDIR}/expat/xmlparse.c
        ${LIBDIR}/expat/xmlrole.c
        ${LIBDIR}/expat/xmltok.c
    )
    set(EXPAT_FOUND 1)
    set(EXPAT_INCLUDE_DIRS ${LIBDIR}/expat/)
    set(EXPAT_LIBRARIES expat)
endif ()

set(GLEW_ROOT "${CMAKE_PREFIX_PATH}")
message("GLEW_ROOT: ${GLEW_ROOT}")
# Find glew or use bundled version
if (SLIC3R_STATIC AND NOT SLIC3R_STATIC_EXCLUDE_GLEW)
    set(GLEW_USE_STATIC_LIBS ON)
    set(GLEW_VERBOSE ON)
endif()

find_package(GLEW REQUIRED)

find_package(glfw3 REQUIRED)

# Find the Cereal serialization library
find_package(cereal REQUIRED)
if (NOT TARGET cereal::cereal)
    set_target_properties(cereal PROPERTIES IMPORTED_GLOBAL TRUE)
    add_library(cereal::cereal ALIAS cereal)
else ()
    set_target_properties(cereal::cereal PROPERTIES IMPORTED_GLOBAL TRUE)
endif ()

# l10n
set(L10N_DIR "${SLIC3R_RESOURCES_DIR}/i18n")
set(BBL_L18N_DIR "${CMAKE_CURRENT_SOURCE_DIR}/localization/i18n")

file(GLOB BBL_L10N_PO_FILES "${BBL_L18N_DIR}/*/OrcaSlicer*.po")
foreach(po_file ${BBL_L10N_PO_FILES})
    GET_FILENAME_COMPONENT(po_dir "${po_file}" DIRECTORY)
    SET(po_new_file "${po_dir}/OrcaSlicer_.po")
    add_custom_command(
#         TARGET gettext_merge_po_with_pot PRE_BUILD
        COMMAND msgmerge -N -o ${po_file} ${po_file} "${BBL_L18N_DIR}/OrcaSlicer.pot"
        DEPENDS ${po_file}
    )
endforeach()

file(GLOB L10N_PO_FILES "${BBL_L18N_DIR}/*/OrcaSlicer*.po")
foreach(po_file ${BBL_L10N_PO_FILES})
    GET_FILENAME_COMPONENT(SECOND_FOLDER_ABSOLUTE ${po_file} DIRECTORY)
    string(REGEX REPLACE ".*/(.*)" "\\1" po_dir "${SECOND_FOLDER_ABSOLUTE}" )
    SET(mo_file "${L10N_DIR}/${po_dir}/OrcaSlicer.mo")

endforeach()

find_package(NLopt 1.4 REQUIRED)

if(SLIC3R_STATIC)
    set(USE_BLOSC TRUE)
endif ()

find_path(SPNAV_INCLUDE_DIR spnav.h)
if (SPNAV_INCLUDE_DIR)
    find_library(HAVE_SPNAV spnav)
    if (HAVE_SPNAV)
        add_definitions(-DHAVE_SPNAV)
        add_library(libspnav SHARED IMPORTED)
        target_link_libraries(libspnav INTERFACE spnav)
        message(STATUS "SPNAV library found")
    else()
        message(STATUS "SPNAV library NOT found, Spacenavd not supported")
    endif()
else()
    message(STATUS "SPNAV library NOT found, Spacenavd not supported")
endif()

add_subdirectory(src)

set(SLIC3R_APP_CMD "orca-slicer")

# Resources install target, configure fhs.hpp on UNIX
if (SLIC3R_FHS)
    # CMAKE_INSTALL_FULL_DATAROOTDIR: read-only architecture-independent data root (share)
    set(SLIC3R_FHS_RESOURCES "${CMAKE_INSTALL_FULL_DATAROOTDIR}/OrcaSlicer")
    install(DIRECTORY ${SLIC3R_RESOURCES_DIR}/ DESTINATION ${SLIC3R_FHS_RESOURCES}
        PATTERN "*/udev" EXCLUDE
    )
    install(FILES src/platform/unix/OrcaSlicer.desktop DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/applications)
    foreach(SIZE 32 128 192)
        install(FILES ${SLIC3R_RESOURCES_DIR}/images/OrcaSlicer_${SIZE}px.png
            DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/icons/hicolor/${SIZE}x${SIZE}/apps RENAME OrcaSlicer.png
        )
    endforeach()
else ()
    install(FILES src/platform/unix/OrcaSlicer.desktop DESTINATION ${CMAKE_INSTALL_PREFIX}/resources/applications)
    install(DIRECTORY "${SLIC3R_RESOURCES_DIR}/" DESTINATION "${CMAKE_INSTALL_PREFIX}/resources")
endif ()

#install(FILES ${CMAKE_SOURCE_DIR}/LICENSE.txt DESTINATION ".")

set (CPACK_PACKAGE_NAME "OrcaSlicer")
set (CPACK_PACKAGE_VENDOR "SoftFever")
set (CPACK_PACKAGE_VERSION_MAJOR "${ORCA_VERSION_MAJOR}")
set (CPACK_PACKAGE_VERSION_MINOR "${ORCA_VERSION_MINOR}")
set (CPACK_PACKAGE_VERSION_PATCH "${ORCA_VERSION_PATCH}")
set (CPACK_PACKAGE_FILE_NAME "OrcaSlicer_Windows_Installer_V${SoftFever_VERSION}")
set (CPACK_PACKAGE_DESCRIPTION_SUMMARY "Orca Slicer is an open source slicer for FDM printers")
set (CPACK_PACKAGE_HOMEPAGE_URL "https://github.com/SoftFever/OrcaSlicer")
set (CPACK_PACKAGE_INSTALL_DIRECTORY ${CPACK_PACKAGE_NAME})
#set (CPACK_PACKAGE_ICON "${CMAKE_SOURCE_DIR}/resources/images\\\\OrcaSlicer.ico")
#set (CPACK_NSIS_MUI_ICON "${CPACK_PACKAGE_ICON}")
#set (CPACK_NSIS_MUI_UNIICON "${CPACK_PACKAGE_ICON}")
set (CPACK_PACKAGE_CHECKSUM SHA256)
#set (CPACK_PACKAGE_INSTALL_REGISTRY_KEY "OrcaSlicer")
#set (CPACK_NSIS_ENABLE_UNINSTALL_BEFORE_INSTALL ON)
#set (CPACK_NSIS_EXECUTABLES_DIRECTORY ".")
set(CPACK_PACKAGE_EXECUTABLES "orca-slicer;OrcaSlicer")
#set(CPACK_CREATE_DESKTOP_LINKS "orca-slicer")
#set(CPACK_GENERATOR NSIS)

