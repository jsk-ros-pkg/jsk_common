cmake_minimum_required(VERSION 2.8.3)
project(dynamic_tf_publisher)

find_package(catkin REQUIRED COMPONENTS geometry_msgs roscpp rospy std_msgs message_generation dynamic_reconfigure)
add_service_files(DIRECTORY srv
  FILES DeleteTF.srv DissocTF.srv SetDynamicTF.srv AssocTF.srv)
generate_messages(
  DEPENDENCIES geometry_msgs
)
generate_dynamic_reconfigure_options(
  cfg/TfParameter.cfg
  )


catkin_package(
    DEPENDS
    CATKIN_DEPENDS geometry_msgs message_runtime
    INCLUDE_DIRS
    LIBRARIES
)

install(PROGRAMS src/tf_publish.py src/tf_set.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY samples 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN ".svn" EXCLUDE)
