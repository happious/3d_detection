cmake_minimum_required(VERSION 3.5)
project(ultralytics_ros)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## ------------------------------------------------------------------
## 1. Find packages
## ------------------------------------------------------------------
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  cv_bridge
  image_transport
  sensor_msgs
  visualization_msgs
  pcl_ros
  pcl_conversions
  message_filters
  tf2_ros
  custom_msgs
)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

## ------------------------------------------------------------------
## 2. 메시지(YoloResult) 제거 (사용 안함)
## ------------------------------------------------------------------
# add_message_files(...)
# generate_messages(...)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
    message_runtime
    custom_msgs
  DEPENDS PCL OpenCV
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## ------------------------------------------------------------------
## 3. C++ Executable (단 하나의 노드만)
## ------------------------------------------------------------------
add_executable(tracker_with_cloud_node
  src/tracker_with_cloud_node.cpp
)

target_link_libraries(tracker_with_cloud_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
)

add_dependencies(tracker_with_cloud_node
  ${catkin_EXPORTED_TARGETS}
)

## ------------------------------------------------------------------
## 4. Python scripts install
## ------------------------------------------------------------------
catkin_install_python(PROGRAMS
  scripts/dino_ocsort.py
  scripts/overlay_test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

