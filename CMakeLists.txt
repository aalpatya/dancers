# 3.15 floor for FetchContent_MakeAvailable (raylib).
cmake_minimum_required(VERSION 3.15)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 20)
# The desktop build uses pthreads/OpenMP; the Emscripten (web) build is single-threaded.
if(NOT EMSCRIPTEN)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
endif()
# Set default build type to release
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose Release or Debug" FORCE)
endif()

# Name of Project
project(dancers)
add_compile_definitions(EIGEN_DONT_PARALLELIZE)
# OpenMP is desktop-only (the web build is single-threaded). OpenCV is no longer used at all - the
# obstacle distance field is computed in C++ (Graphics::buildObstacleGround) on every platform.
if(NOT EMSCRIPTEN)
    find_package(OpenMP REQUIRED)
endif()

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
# Build raylib for the web (GLFW + WebGL via OpenGL ES2) when targeting Emscripten.
if(EMSCRIPTEN)
    set(PLATFORM Web CACHE STRING "" FORCE)
endif()
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

# Baked-in path definitions (resolve from anywhere). PROJECT_ROOT: on the web it's the mount point of
# the preloaded virtual filesystem (see the --preload-file flags). GLSL_DIR: the shader dialect subdir
# under assets/shaders ("glsl330" for desktop GL, "glsl100" for WebGL ES2).
if(EMSCRIPTEN)
    set(GBP_PROJECT_ROOT "/app")
    set(GBP_GLSL_DIR "glsl100")
else()
    set(GBP_PROJECT_ROOT "${CMAKE_SOURCE_DIR}")
    set(GBP_GLSL_DIR "glsl330")
endif()
target_compile_definitions(gbp PUBLIC PROJECT_ROOT="${GBP_PROJECT_ROOT}" GLSL_DIR="${GBP_GLSL_DIR}")
target_include_directories(gbp PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/inc)
target_link_libraries(gbp PUBLIC raylib Eigen3::Eigen manif)
if(NOT EMSCRIPTEN)
    target_link_libraries(gbp PUBLIC OpenMP::OpenMP_CXX)   # desktop: parallel GBP over robots
endif()

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

# Checks if OSX and links appropriate frameworks (only required on MacOS, not for the web build)
if (APPLE AND NOT EMSCRIPTEN)
    target_link_libraries(
      dancers PUBLIC
          "-framework Cocoa, -framework OpenGL, -framework IOKit, -framework CoreVideo"
      )
endif()

# Web build: emit dancers.html/.js/.wasm, drive raylib's GLFW/WebGL backend, and preload the config and
# the runtime assets (shaders + models) into the virtual filesystem under /app (= PROJECT_ROOT above).
if(EMSCRIPTEN)
    # The preloaded config files are baked into dancers.data at link time, so editing one must trigger a
    # relink. List them (and the shell) as LINK_DEPENDS; CONFIGURE_DEPENDS re-globs them each build.
    file(GLOB_RECURSE DANCERS_CONFIGS CONFIGURE_DEPENDS ${CMAKE_SOURCE_DIR}/config/*.yaml)
    # OUTPUT_NAME dancers + SUFFIX .html -> emit dancers.html/.js/.wasm/.data. The front page (index.html,
    # copied below) is the folder-URL landing; the multirobot demos load dancers.html in an iframe.
    # LINK_DEPENDS -> relink when the shell or any config changes.
    set_target_properties(dancers PROPERTIES OUTPUT_NAME "dancers" SUFFIX ".html"
        LINK_DEPENDS "${CMAKE_SOURCE_DIR}/assets/web/embed_shell.html;${DANCERS_CONFIGS}")
    target_link_options(dancers PRIVATE
        -sUSE_GLFW=3
        -sALLOW_MEMORY_GROWTH=1
        -sFORCE_FILESYSTEM=1
        -sASSERTIONS=1
        -sGL_ENABLE_GET_PROC_ADDRESS
        "SHELL:-sEXPORTED_RUNTIME_METHODS=['FS']"   # let the shell read/write the config in the virtual FS
        "SHELL:--shell-file ${CMAKE_SOURCE_DIR}/assets/web/embed_shell.html"   # minimal canvas (the demos page is the front door)
        "SHELL:--preload-file ${CMAKE_SOURCE_DIR}/config@/app/config"
        "SHELL:--preload-file ${CMAKE_SOURCE_DIR}/assets/shaders@/app/assets/shaders"
        "SHELL:--preload-file ${CMAKE_SOURCE_DIR}/assets/models@/app/assets/models"
        "SHELL:--preload-file ${CMAKE_SOURCE_DIR}/assets/imgs@/app/assets/imgs"
    )

    # The visual gbp examples, each built as its own web page (gbp-1d-line-fitting.html,
    # gbp-se2-localisation.html). They need no preloaded assets (raylib's embedded default font only).
    # assets/web/index.html is the front page that embeds the selected demo (multirobot or gbp) in an iframe.
    foreach(demo gbp-1d-line-fitting gbp-se2-localisation)
        add_executable(${demo} src/examples/${demo}.cpp)
        target_link_libraries(${demo} PUBLIC gbp Eigen3::Eigen)
        set_target_properties(${demo} PROPERTIES SUFFIX ".html"
            LINK_DEPENDS "${CMAKE_SOURCE_DIR}/assets/web/gbp_demo_shell.html")
        target_link_options(${demo} PRIVATE
            -sUSE_GLFW=3
            -sALLOW_MEMORY_GROWTH=1
            -sASSERTIONS=1
            -sGL_ENABLE_GET_PROC_ADDRESS
            "SHELL:--shell-file ${CMAKE_SOURCE_DIR}/assets/web/gbp_demo_shell.html"
        )
    endforeach()
    # Put the front page alongside the build outputs in build_web/ as index.html (the folder-URL landing).
    configure_file(${CMAKE_SOURCE_DIR}/assets/web/index.html ${CMAKE_BINARY_DIR}/index.html COPYONLY)
endif()

#################################################################
# Standalone executables (desktop-only, not part of the web app). The main app `dancers` lands in
# build/; these are grouped by kind: the gbp examples in build/examples/, helper tools in build/helpers/.
if(NOT EMSCRIPTEN)
    # ── GBP examples (build/examples/) ────────────────────────────────────────
    # Each links just the gbp library (no Globals/Utils - the library no longer reads the app's globals).
    add_executable(gbp-three-variable-factorgraph   src/examples/gbp-three-variable-factorgraph.cpp)
    add_executable(gbp-two-layer-factorgraph src/examples/gbp-two-layer-factorgraph.cpp)
    add_executable(gbp-1d-line-fitting       src/examples/gbp-1d-line-fitting.cpp)
    add_executable(gbp-se2-localisation      src/examples/gbp-se2-localisation.cpp)
    foreach(ex gbp-three-variable-factorgraph gbp-two-layer-factorgraph gbp-1d-line-fitting gbp-se2-localisation)
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
endif()  # NOT EMSCRIPTEN