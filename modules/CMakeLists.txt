
# OpenCV used to read images, and keypoint detection/matching
find_package(OpenCV COMPONENTS calib3d imgproc highgui core REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})


add_subdirectory(io)
add_subdirectory(filtering)
add_subdirectory(surface)
add_subdirectory(image)
add_subdirectory(optimization)
