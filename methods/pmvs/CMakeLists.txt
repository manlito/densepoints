project(pmvs)
add_definitions(-std=c++11)

include_directories(.)
aux_source_directory(. PMVS_SOURCES)
add_library(${PROJECT_NAME} ${PMVS_SOURCES})

# OpenCV
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

# PCL
target_link_libraries(${PROJECT_NAME} ${PCL_COMMON_LIBRARIES})
add_definitions(${PCL_DEFINITIONS})

# Densepoints common definitions and libraries
target_link_libraries(${PROJECT_NAME} ${DENSEPOINTS_LIBS})
add_definitions(${DENSEPOINTS_DEFINITIONS})
