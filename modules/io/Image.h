project(densepoints_io)

include_directories(.)
aux_source_directory(. DENSEPOINTS_IO_SOURCES)
add_library(${PROJECT_NAME} ${DENSEPOINTS_IO_SOURCES})

// OpenCV for image reading
find_package(OpenCV COMPONENTS core REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
