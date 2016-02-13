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
find_package(OpenCV COMPONENTS features2d calib3d imgproc highgui core REQUIRED)
message(STATUS "OpenCVIncludes: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCVLibs: ${OpenCV_LIBS}")

# Dependency: PCL.
# For point set efficient search
find_package(PCL COMPONENTS filter geometry common REQUIRED)
message(STATUS "PCLIncludes: ${PCL_INCLUDE_DIRS}")
message(STATUS "PCLLibs: ${PCL_LIBRARIES}")

# Dependency: Threads
find_package(Threads)
message(STATUS "ThreadsLibs: ${CMAKE_THREAD_LIBS_INIT}")

# Common libraries all projects need to be linked with
set(DENSEPOINTS_DEFINITIONS -DELPP_THREAD_SAFE -DELPP_FORCE_USE_STD_THREAD)
set(DENSEPOINTS_LIBS ${CMAKE_THREAD_LIBS_INIT})

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
