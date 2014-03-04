cmake_minimum_required(VERSION 2.8.3)
project(rospatlite)

catkin_package(
  DEPENDS
  CATKIN_DEPENDS std_msgs rospy
  )

install(DIRECTORY launch scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)
