project(densepoints_io)

include_directories(.)
aux_source_directory(. DENSEPOINTS_IO_SOURCES)
add_library(${PROJECT_NAME} ${DENSEPOINTS_IO_SOURCES})

# Densepoints common definitions and libraries
target_link_libraries(${PROJECT_NAME} ${DENSEPOINTS_LIBS})
add_definitions(${DENSEPOINTS_DEFINITIONS})
