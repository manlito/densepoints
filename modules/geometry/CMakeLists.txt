project(densepoints_geometry)

include_directories(.)
aux_source_directory(. DENSEPOINTS_GEOMETRY_SOURCES)
add_library(${PROJECT_NAME} ${DENSEPOINTS_GEOMETRY_SOURCES})

find_package(Eigen REQUIRED)
include_directories(EIGEN_INCLUDE_DIRS)

# Densepoints common definitions and libraries
target_link_libraries(${PROJECT_NAME} ${DENSEPOINTS_LIBS})
add_definitions(${DENSEPOINTS_DEFINITIONS})
