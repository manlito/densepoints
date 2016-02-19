project(densify)

add_executable(${PROJECT_NAME} main.cpp)

# OpenCV
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

# PCL
target_link_libraries(${PROJECT_NAME} ${PCL_COMMON_LIBRARIES})
add_definitions(${PCL_DEFINITIONS})

# DensePoints::PMVS
target_link_libraries(${PROJECT_NAME} pmvs)

# Densepoints common definitions and libraries
target_link_libraries(${PROJECT_NAME} ${DENSEPOINTS_LIBS})
add_definitions(${DENSEPOINTS_DEFINITIONS})
