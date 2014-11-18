cmake_minimum_required(VERSION 2.8.3)
project(jsk_network_tools)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

add_message_files(
  DIRECTORY msg
  FILES
  Heartbeat.msg
  HeartbeatResponse.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)
catkin_package(
  DEPENDS
  CATKIN_DEPENDS std_msgs message_runtime
#  INCLUDE_DIRS include
#  LIBRARIES jsk_network_tools
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

install(DIRECTORY scripts
  DESTINATION
  ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  )
