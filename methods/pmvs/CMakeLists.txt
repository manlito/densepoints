project(pmvs)
add_definitions(-std=c++11)

include_directories(.)
aux_source_directory(. PMVS_SOURCES)
add_library(${PROJECT_NAME} ${PMVS_SOURCES})

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

target_link_libraries(${PROJECT_NAME} ${PCL_COMMON_LIBRARIES})
add_definitions(${PCL_DEFINITIONS})
