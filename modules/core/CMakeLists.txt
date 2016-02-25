project(densepoints_core)

include_directories(.)
aux_source_directory(. DENSEPOINTS_CORE_SOURCES)
add_library(${PROJECT_NAME} ${DENSEPOINTS_CORE_SOURCES})

find_package(Eigen REQUIRED)
include_directories(EIGEN_INCLUDE_DIRS)

# OpenCV
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

# Densepoints common definitions and libraries
target_link_libraries(${PROJECT_NAME} ${DENSEPOINTS_THIRD_PARTYLIBS})
add_definitions(${DENSEPOINTS_DEFINITIONS})
