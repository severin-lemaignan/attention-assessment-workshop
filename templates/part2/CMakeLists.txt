cmake_minimum_required(VERSION 2.8.3)
project(attention_assessment)

add_definitions(-std=c++11)

set(DLIB_PATH "" CACHE PATH "Path to DLIB")
include(${DLIB_PATH}/dlib/cmake)

find_package(OpenCV COMPONENTS core imgproc calib3d highgui REQUIRED)
message(STATUS "OpenCV version: ${OpenCV_VERSION}")

include_directories(${OpenCV_INCLUDE_DIRS})

# Create the library
add_library(head_pose_estimation SHARED src/head_pose_estimation.cpp)
target_link_libraries(head_pose_estimation dlib ${OpenCV_LIBRARIES})

# Create the executable binary
add_executable(head_detection src/head_detection.cpp)
target_link_libraries(head_detection head_pose_estimation ${OpenCV_LIBRARIES})

##### ROS ######

find_package(catkin REQUIRED COMPONENTS 
    roscpp 
    tf
    std_msgs
    visualization_msgs
    sensor_msgs
    cv_bridge
    image_transport
    image_geometry
)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
    CATKIN_DEPENDS tf
    DEPENDS OpenCV
    LIBRARIES 
)

add_executable(head_pose_estimator src/ros_node.cpp)
target_link_libraries(head_pose_estimator head_pose_estimation ${catkin_LIBRARIES})

install(TARGETS head_pose_estimation head_pose_estimator
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
    launch/attention_assessment.launch
    share/shape_predictor_68_face_landmarks.dat
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

