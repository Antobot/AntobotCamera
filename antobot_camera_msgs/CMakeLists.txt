cmake_minimum_required(VERSION 3.8)
project(antobot_camera_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# ------------------------------
# Message / Service / Action Files
# ------------------------------
set(MSG_FILES
  "msg/TransferStatus.msg"
  "msg/SessionTransferItem.msg"
)

set(SRV_FILES
  "srv/CameraRecord.srv"
  "srv/CamManager.srv"
  "srv/DeleteSessions.srv"
  "srv/GetAvailability.srv"
  "srv/GetBandwidth.srv"
  "srv/GetSessionTransferList.srv"
)

set(ACTION_FILES
  "action/DoTransfer.action"
)

# ------------------------------
# Generate ROS interfaces
# ------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  ${SRV_FILES}
  ${ACTION_FILES}
  DEPENDENCIES std_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
