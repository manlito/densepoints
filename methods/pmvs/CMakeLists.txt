project(pmvs)

include_directories(.)
aux_source_directory(. PMVS_SOURCES)
add_library(${PROJECT_NAME} ${PMVS_SOURCES})

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
