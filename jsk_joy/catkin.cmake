cmake_minimum_required(VERSION 2.8.3)
project(jsk_joy)

find_package(catkin REQUIRED COMPONENTS ps3joy tf view_controller_msgs interactive_markers visualization_msgs jsk_footstep_msgs) # pr2_controllers_msgs robot_monitor
catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS ps3joy tf view_controller_msgs interactive_markers visualization_msgs jsk_footstep_msgs
  
  )

install(DIRECTORY launch scripts configs
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  PATTERN ".svn" EXCLUDE
  )

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


