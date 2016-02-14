project(stlplus)

include_directories(.)
aux_source_directory(. STL_SOURCES)
add_library(${PROJECT_NAME} ${STL_SOURCES})

