cmake_minimum_required(VERSION 2.8.3)
project(jsk_network_tools)

find_package(catkin REQUIRED COMPONENTS
  message_generation std_msgs sensor_msgs rostest
  dynamic_reconfigure)
catkin_python_setup()
add_message_files(
  DIRECTORY msg
  FILES
  CompressedAngleVectorPR2.msg
  Heartbeat.msg
  HeartbeatResponse.msg
  FC2OCS.msg
  OCS2FC.msg
  FC2OCSLargeData.msg
  OpenNISample.msg
  AllTypeTest.msg
)
add_service_files(
  FILES
  SetSendRate.srv
)
generate_dynamic_reconfigure_options(
  cfg/SilverhammerHighspeedStreamer.cfg
  )
## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs sensor_msgs
)
catkin_package(
  DEPENDS
  CATKIN_DEPENDS std_msgs message_runtime
#  INCLUDE_DIRS include
#  LIBRARIES jsk_network_tools
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

add_rostest(test/test_all_type_low_speed.test)

install(DIRECTORY scripts
  DESTINATION
  ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  )
