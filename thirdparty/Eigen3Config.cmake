# Minimal config package for the header-only Eigen vendored next to this file (thirdparty/Eigen).
# The headers are located relative to this file (CMAKE_CURRENT_LIST_DIR), so it works on any platform
# and checkout location without fetching or building Eigen.
set(EIGEN3_INCLUDE_DIR  "${CMAKE_CURRENT_LIST_DIR}")
set(EIGEN3_INCLUDE_DIRS "${CMAKE_CURRENT_LIST_DIR}")
if(NOT TARGET Eigen3::Eigen)
    add_library(Eigen3::Eigen INTERFACE IMPORTED)
    set_target_properties(Eigen3::Eigen PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${EIGEN3_INCLUDE_DIR}")
endif()
set(Eigen3_FOUND TRUE)
