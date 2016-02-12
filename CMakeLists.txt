cmake_minimum_required(VERSION 2.8)

project(densification)

# FindXXX.cmake files
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/build/cmake")

# Options
option (Densification_USE_OPENMP "Enable OpenMP for multicore usage" ON)

set(DENSIFITION_VERSION_MAJOR 0)
set(DENSIFITION_VERSION_MINOR 1)
set(DENSIFITION_VERSION_PATCH 0)

# Dependency: OpenCV.
# We use OpenCV as the backend for all image operations.
find_package(OpenCV COMPONENTS calib3d imgproc highgui core REQUIRED)
message(STATUS "OpenCVIncludes: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCVLibs: ${OpenCV_LIBS}")

# Dependency: CGAL.
# For point set efficient search
find_package(CGAL COMPONENTS core REQUIRED)
message(STATUS "CGALUseFile: ${CGAL_USE_FILE}")
include (${CGAL_USE_FILE})

# Dependency: TBB (Optional)
find_package(TBB QUIET)
if(TBB_FOUND AND NOT APPLE)
    include(${TBB_USE_FILE})
    list(APPEND CGAL_3RD_PARTY_LIBRARIES ${TBB_LIBRARIES})
endif()
message(STATUS "CGALLibs: ${CGAL_LIBRARIES} ${GMP_LIBRARIES} ${MPFR_LIBRARIES}")

# Default build type: Release
if (NOT CMAKE_BUILD_TYPE AND NOT MSVC)
  set(CMAKE_BUILD_TYPE "Release")
endif ()

# Libraries most likely to be frozen
add_subdirectory(third_party)

# Shared functionality across methos
add_subdirectory(modules)

# Densification methods implemented
add_subdirectory(methods)

# A frontend programs for usage and interoperation
add_subdirectory(programs)
