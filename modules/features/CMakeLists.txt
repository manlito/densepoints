project(densepoints_features)

include_directories(.)
aux_source_directory(. DENSEPOINTS_FEATURES_SOURCES)
add_library(${PROJECT_NAME} ${DENSEPOINTS_FEATURES_SOURCES})

find_package(Eigen REQUIRED)
include_directories(EIGEN_INCLUDE_DIRS)

# OpenCV
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

# Densepoints common definitions and libraries
target_link_libraries(${PROJECT_NAME} ${DENSEPOINTS_THIRD_PARTYLIBS})
add_definitions(${DENSEPOINTS_DEFINITIONS})
