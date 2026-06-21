# 3.15 floor for FetchContent_MakeAvailable (raylib).
cmake_minimum_required(VERSION 3.15)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 20)
# The desktop build uses pthreads/OpenMP.
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
# Set default build type to release
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose Release or Debug" FORCE)
endif()

# Name of Project
project(dancers)
add_compile_definitions(EIGEN_DONT_PARALLELIZE)
# OpenCV is no longer used at all - the obstacle distance field is computed in C++
# (Graphics::buildObstacleGround).
find_package(OpenMP REQUIRED)

# ── Fetch third-party dependencies ────────────────────────────────────────────
include(FetchContent)
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)   # don't build deps' test suites

# Eigen — header-only, vendored in thirdparty/Eigen with a committed config package
# (thirdparty/Eigen3Config.cmake). Point Eigen3_DIR at it so find_package(Eigen3) resolves the headers
# on any platform without fetching or building. find_package defines the Eigen3::Eigen target; smooth's
# internal find_package(Eigen3) reuses the same config.
set(Eigen3_DIR "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty" CACHE PATH "" FORCE)
find_package(Eigen3 REQUIRED)

# manif — Lie group library (header-only, depends on Eigen3; bundles its own tl-optional). Pinned to a
# release tag for reproducibility; the `manif` interface target carries the include paths.
# manif's CMake reads EIGEN3_VERSION (set by the module-mode FindEigen3); our config-mode Eigen3 doesn't
# define it, so set it here (our vendored Eigen is 3.4.0) to satisfy manif's version check.
set(EIGEN3_VERSION "3.4.0")
FetchContent_Declare(
    manif
    GIT_REPOSITORY https://github.com/artivis/manif.git
    GIT_TAG 0.0.5
    GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(manif)

# raylib — windowing / graphics. Examples/games are off so only the static library
# is built. The previously-vendored fork's only custom addition, DrawTextureFlat,
# now lives in src/Graphics.cpp, so we can build against clean upstream raylib.
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_GAMES OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
    raylib
    GIT_REPOSITORY https://github.com/raysan5/raylib.git
    GIT_TAG 5.5
    GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(raylib)
# raylib's own sources (miniaudio, stb_vorbis, system AudioToolbox headers) emit
# warnings we don't control — silence them so they don't drown out our own.
if(TARGET raylib)
    target_compile_options(raylib PRIVATE -w)
endif()

#################################################################

add_library(gbp STATIC
src/gbp/Variable.cpp
src/gbp/Factor.cpp
)

# Baked-in path definitions (resolve from anywhere). GLSL_DIR: the shader dialect subdir under
# assets/shaders ("glsl330" for desktop GL).
set(GBP_PROJECT_ROOT "${CMAKE_SOURCE_DIR}")
set(GBP_GLSL_DIR "glsl330")
target_compile_definitions(gbp PUBLIC PROJECT_ROOT="${GBP_PROJECT_ROOT}" GLSL_DIR="${GBP_GLSL_DIR}")
target_include_directories(gbp PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/inc)
target_link_libraries(gbp PUBLIC raylib Eigen3::Eigen manif)
target_link_libraries(gbp PUBLIC OpenMP::OpenMP_CXX)   # parallel GBP over robots

# Header-only third-party libs (json.hpp, raygui.h, nanoflann.h, rlights.h, …) are
# treated as SYSTEM includes so their warnings are suppressed while our own code is
# still fully diagnosed. PUBLIC so it propagates to the dancers target too.
target_include_directories(gbp SYSTEM PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty)

# Every layer in src/CustomFactorGraphLayers/ is compiled automatically (CONFIGURE_DEPENDS re-globs each
# build), so a new layer just needs its .cpp dropped in there - no edit to this file.
file(GLOB FGLAYER_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/CustomFactorGraphLayers/*.cpp)

add_executable(dancers
    src/main.cpp
    src/Globals.cpp
    src/Robot.cpp
    ${FGLAYER_SOURCES}
    src/ShapeFormation.cpp
    src/Simulator.cpp
    src/PathPlanningScenarios.cpp
    src/Utils.cpp
    src/Graphics.cpp
)
target_link_libraries(dancers PUBLIC gbp Eigen3::Eigen)

# Checks if OSX and links appropriate frameworks (only required on MacOS)
if (APPLE)
    target_link_libraries(
      dancers PUBLIC
          "-framework Cocoa, -framework OpenGL, -framework IOKit, -framework CoreVideo"
      )
endif()

#################################################################
# Standalone executables. The main app `dancers` lands in build/; these are grouped by kind: the gbp
# examples in build/examples/, helper tools in build/helpers/.

# ── GBP examples (build/examples/) ────────────────────────────────────────
# Each links just the gbp library (no Globals/Utils - the library no longer reads the app's globals).
add_executable(gbp-three-variable-factorgraph   src/examples/gbp-three-variable-factorgraph.cpp)
add_executable(gbp-two-layer-factorgraph src/examples/gbp-two-layer-factorgraph.cpp)
add_executable(gbp-1d-line-fitting       src/examples/gbp-1d-line-fitting.cpp)
foreach(ex gbp-three-variable-factorgraph gbp-two-layer-factorgraph gbp-1d-line-fitting)
    target_link_libraries(${ex} PUBLIC gbp Eigen3::Eigen)
    set_target_properties(${ex} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/examples)
    if (APPLE)
        # raylib is linked transitively through gbp, so its system frameworks are needed even for the
        # headless demos.
        target_link_libraries(${ex} PUBLIC
            "-framework Cocoa, -framework OpenGL, -framework IOKit, -framework CoreVideo")
    endif()
endforeach()

# ── Helper tools (build/helpers/) ─────────────────────────────────────────
# Binary shape-image editor (GUI). Standalone: just raylib + raygui (from thirdparty), no gbp.
add_executable(shape_editor src/helpers/shape_editor.cpp)
target_link_libraries(shape_editor PUBLIC raylib)
target_include_directories(shape_editor SYSTEM PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty)
target_compile_definitions(shape_editor PRIVATE PROJECT_ROOT="${CMAKE_SOURCE_DIR}")
set_target_properties(shape_editor PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/helpers)
if (APPLE)
    target_link_libraries(shape_editor PUBLIC
        "-framework Cocoa, -framework OpenGL, -framework IOKit, -framework CoreVideo")
endif()
