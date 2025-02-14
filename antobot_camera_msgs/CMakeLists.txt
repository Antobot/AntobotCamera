cmake_minimum_required(VERSION 2.8.3)
project(antobot_camera_msgs)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

# add_message_files(
#   FILES
# )

add_service_files(
  FILES
  camManager.srv
  cameraRecord.srv
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS 
    std_msgs
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS})
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)



