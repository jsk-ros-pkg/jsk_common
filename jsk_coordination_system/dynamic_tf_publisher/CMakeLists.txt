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
install(PROGRAMS src/tf_publish.py src/tf_set.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY samples
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN ".svn" EXCLUDE)
