project(densepoints_image)

include_directories(.)
aux_source_directory(. DENSEPOINTS_IMAGE_SOURCES)
add_library(${PROJECT_NAME} ${DENSEPOINTS_IMAGE_SOURCES})

