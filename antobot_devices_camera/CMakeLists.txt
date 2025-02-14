cmake_minimum_required(VERSION 3.0.2)
project(antobot_devices_camera)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs 
  tf
  geonav_transform
  tf2
  tf2_ros
  std_msgs
)

catkin_package(
   CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs tf geonav_transform tf2 tf2_ros std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
