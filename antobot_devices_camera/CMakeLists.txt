cmake_minimum_required(VERSION 3.0.2)
project(antobot_devices_camera)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs 
  tf
  tf2
  tf2_ros
  std_msgs
)


catkin_package(
   CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs tf tf2 tf2_ros std_msgs
)

catkin_install_python(PROGRAMS
  src/camera_record.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_python_setup()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
