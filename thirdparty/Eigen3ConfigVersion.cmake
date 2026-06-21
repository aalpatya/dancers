# Version file for the vendored Eigen (thirdparty/Eigen, Eigen 3.4.x). Reports compatibility for any
# find_package(Eigen3 <ver>) request up to this version (e.g. smooth asks for 3.4).
set(PACKAGE_VERSION "3.4.0")
set(PACKAGE_VERSION_COMPATIBLE TRUE)
if(PACKAGE_FIND_VERSION VERSION_GREATER PACKAGE_VERSION)
    set(PACKAGE_VERSION_COMPATIBLE FALSE)
endif()
if(PACKAGE_FIND_VERSION VERSION_EQUAL PACKAGE_VERSION)
    set(PACKAGE_VERSION_EXACT TRUE)
endif()
