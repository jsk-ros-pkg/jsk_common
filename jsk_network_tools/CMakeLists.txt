cmake_minimum_required(VERSION 2.8.3)
project(jsk_network_tools)

find_package(catkin REQUIRED COMPONENTS
  message_generation std_msgs sensor_msgs rostest
  dynamic_reconfigure roscpp)
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
  SilverhammerInternalBuffer.msg
  WifiStatus.msg
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
add_definitions("-O2")
include_directories(include ${catkin_INCLUDE_DIRS})
link_libraries(${catkin_LIBRARIES})
add_executable(silverhammer_highspeed_internal_receiver
  src/silverhammer_highspeed_internal_receiver.cpp)
add_dependencies(silverhammer_highspeed_internal_receiver
  ${PROJECT_NAME}_gencpp ${PROJECT_NAME}_gencfg)

# Sometimes rostest fails to detect jsk_network_tools package on jenkins.
# So we remove this test.
# if(CATKIN_ENABLE_TESTING)
#   add_rostest(test/test_all_type_low_speed.test)
# endif(CATKIN_ENABLE_TESTING)

# From https://github.com/jsk-ros-pkg/jsk_recognition/pull/2345
# Install header files directly into ${CATKIN_PACKAGE_INCLUDE_DESTINATION}.
# If the package has setup.py and modules under src/${PROJECT_NAME}/,
# install python executables directly into ${CATKIN_PACKAGE_BIN_DESTINATION}.
# However, if it doesn't have setup.py, directories including python executables
# should be installed recursively into ${CATKIN_PACKAGE_SHARE_DESTINATION}.
# Also, other directories like 'launch' or 'sample' must be installed
# recursively into ${CATKIN_PACKAGE_SHARE_DESTINATION}.
# Be careful that 'launch' and 'launch/' are different: the former is directory
# and the latter is each content.
install(DIRECTORY scripts/
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
install(DIRECTORY launch test
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
install(TARGETS silverhammer_highspeed_internal_receiver
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
