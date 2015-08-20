cmake_minimum_required(VERSION 2.8.3)
project(attention_assessment)

add_definitions(-std=c++11)

set(DLIB_PATH "" CACHE PATH "Path to DLIB")
include(${DLIB_PATH}/dlib/cmake)

find_package(OpenCV COMPONENTS core imgproc calib3d highgui REQUIRED)
message(STATUS "OpenCV version: ${OpenCV_VERSION}")

include_directories(${OpenCV_INCLUDE_DIRS})

# Create the library
add_library(head_pose_estimation SHARED head_pose_estimation.cpp)
target_link_libraries(head_pose_estimation dlib ${OpenCV_LIBRARIES})

# Create the executable binary
add_executable(head_detection head_detection.cpp)
target_link_libraries(head_detection head_pose_estimation ${OpenCV_LIBRARIES})



