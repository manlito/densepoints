project(densepoints_io)

include_directories(.)
aux_source_directory(. DENSEPOINTS_IO_SOURCES)
add_library(${PROJECT_NAME} ${DENSEPOINTS_IO_SOURCES})

# DensePoints::Core
target_link_libraries(${PROJECT_NAME} densepoints_core)

# Densepoints common definitions and libraries
target_link_libraries(${PROJECT_NAME} ${DENSEPOINTS_LIBS})
add_definitions(${DENSEPOINTS_DEFINITIONS})
