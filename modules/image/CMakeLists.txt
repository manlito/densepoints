project(densepoints_io)

include_directories(.)
aux_source_directory(. DENSEPOINTS_IO_SOURCES)
add_library(${PROJECT_NAME} ${DENSEPOINTS_IO_SOURCES})

