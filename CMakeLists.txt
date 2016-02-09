cmake_minimum_required(VERSION 2.8)

project(densification)

# Options
option (Densification_USE_OPENMP "Enable OpenMP for multicore usage" ON)

set(DENSIFITION_VERSION_MAJOR 0)
set(DENSIFITION_VERSION_MINOR 1)
set(DENSIFITION_VERSION_PATCH 0)

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
