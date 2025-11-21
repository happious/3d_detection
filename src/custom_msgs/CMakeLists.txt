cmake_minimum_required(VERSION 3.0.2)
project(custom_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  message_generation
)

add_message_files(
  FILES
  BoundingBox2D.msg
  BoundingBox3D.msg
  ObjectHypothesis.msg
  Detection3D.msg
  Detection3DArray.msg
  TrackingResult.msg
  TrackingResultArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs
)

