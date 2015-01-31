cmake_minimum_required(VERSION 2.8.3)
project(jsk_tools)

find_package(catkin REQUIRED)
set(ROS_BUILD_TYPE RelWithDebInfo)
catkin_package(
  CATKIN_DEPENDS #
  LIBRARIES #
  INCLUDE_DIRS #
  DEPENDS #
  CFG_EXTRAS download_package.cmake
  )

install(DIRECTORY src test bin
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
