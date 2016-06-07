cmake_minimum_required(VERSION 2.8)

project(densification)

# FindXXX.cmake files
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/build/cmake")

# Options
option(DENSEPOINTS_USE_OPENMP "Enable OpenMP for multicore usage" ON)
option(DENSEPOINTS_BUILD_TESTS "Build tests" ON)
option(DENSEPOINTS_DEBUG_PMVS_SEEDS "Print images for visualization on PMVS::Seed" OFF)
option(DENSEPOINTS_DEBUG_PMVS_OPTIMIZATION "Print patches for visualization on PMVS::Optimization" OFF)
set(DENSEPOINTS_DEBUG_OUTPUT_PATH "${CMAKE_CURRENT_BINARY_DIR}/debug_data" CACHE STRING "Folder where debug results will be put")
file(MAKE_DIRECTORY ${DENSEPOINTS_DEBUG_OUTPUT_PATH})

set(DENSIFITION_VERSION_MAJOR 0)
set(DENSIFITION_VERSION_MINOR 1)
set(DENSIFITION_VERSION_PATCH 0)

# Dependency: OpenMP
find_package(OpenMP QUIET)
if(OPENMP_FOUND)
  message(STATUS "OpenMP Flags: ${OpenMP_CXX_FLAGS}")
endif()

# Dependency: OpenCV
# We use OpenCV as the backend for all image operations.
find_package(OpenCV COMPONENTS features2d xfeatures2d calib3d imgproc highgui core REQUIRED)
message(STATUS "OpenCVIncludes: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCVLibs: ${OpenCV_LIBS}")

# Dependency: PCL
# For point set efficient search
find_package(PCL COMPONENTS filter geometry common REQUIRED)
message(STATUS "PCLIncludes: ${PCL_INCLUDE_DIRS}")
message(STATUS "PCLLibs: ${PCL_LIBRARIES}")

# Dependency: Threads
find_package(Threads)
message(STATUS "ThreadsLibs: ${CMAKE_THREAD_LIBS_INIT}")

# DensePoints: Definitions
set(DENSEPOINTS_DEFINITIONS -std=c++11 -DELPP_THREAD_SAFE -DELPP_FORCE_USE_STD_THREAD -DDEBUG_OUTPUT_PATH="${DENSEPOINTS_DEBUG_OUTPUT_PATH}")
if(DENSEPOINTS_DEBUG_PMVS_SEEDS)
  message(STATUS "DEBUG: PMVS::Seeds enabled")
  set(DENSEPOINTS_DEFINITIONS ${DENSEPOINTS_DEFINITIONS} -DDEBUG_PMVS_SEEDS)
endif()
if(DENSEPOINTS_DEBUG_PMVS_OPTIMIZATION)
  message(STATUS "DEBUG: PMVS::Optimization enabled")
  set(DENSEPOINTS_DEFINITIONS ${DENSEPOINTS_DEFINITIONS} -DDEBUG_PMVS_OPTIMIZATION)
endif()
# DensePoints: Libraries
set(DENSEPOINTS_LIBS
    densepoints_io
    densepoints_geometry
    densepoints_core
)
# DensePoints: Libraries
set(DENSEPOINTS_THIRD_PARTY_LIBS
    ${CMAKE_THREAD_LIBS_INIT}
    ${OpenCV_LIBS}
    stlplus
    rplycpp
    rply
)
if(OPENMP_FOUND)
    set(DENSEPOINTS_DEFINITIONS ${DENSEPOINTS_DEFINITIONS} ${OpenMP_CXX_FLAGS})
    set(DENSEPOINTS_THIRD_PARTY_LIBS ${DENSEPOINTS_THIRD_PARTY_LIBS} ${OpenMP_CXX_FLAGS})
endif()

# Default build type: Release
if (NOT CMAKE_BUILD_TYPE AND NOT MSVC)
  set(CMAKE_BUILD_TYPE "Release")
endif ()


# DensePoints include directories
include_directories(
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  "${CMAKE_CURRENT_SOURCE_DIR}/third_party"
  "${CMAKE_CURRENT_SOURCE_DIR}/modules"
  "${CMAKE_CURRENT_SOURCE_DIR}/methods"
)

# Libraries most likely to be frozen
add_subdirectory(third_party)

# Shared functionality across methos
add_subdirectory(modules)

# Densification methods implemented
add_subdirectory(methods)

# A frontend programs for usage and interoperation
add_subdirectory(programs)

# Add tests when possible
if (DENSEPOINTS_BUILD_TESTS)
  enable_testing()
  find_package(GTest QUIET)
  if(GTEST_FOUND)
    message(STATUS "GTestIncludes: ${GTEST_INCLUDE_DIRS}")
    message(STATUS "GTestLibraries: ${GTEST_LIBRARIES}")
  else()
    message(STATUS "GTest could not be found")
  endif()
endif()

if(GTEST_FOUND)
  add_subdirectory(tests)
endif()
