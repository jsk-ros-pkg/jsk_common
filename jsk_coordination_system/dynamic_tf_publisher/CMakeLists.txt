# Fall back to ROS1 when $ENV{ROS_VERSION} is unset (pre-ros_environment distros,
# e.g. indigo); every ROS2 environment always sets it via ros_environment.
if("$ENV{ROS_VERSION}" STREQUAL "")
  set(ENV{ROS_VERSION} 1)
endif()

# cmake_minimum_required() must run before project(), so it needs its own
# ROS_VERSION branch here (indigo's cmake is too old for VERSION 3.8).
if("$ENV{ROS_VERSION}" EQUAL 1)
cmake_minimum_required(VERSION 2.8.3)
elseif("$ENV{ROS_VERSION}" EQUAL 2)
  cmake_minimum_required(VERSION 3.8)
endif()

project(dynamic_tf_publisher)

if("$ENV{ROS_VERSION}" EQUAL 1)
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
elseif("$ENV{ROS_VERSION}" EQUAL 2)
  find_package(ament_cmake REQUIRED)
  find_package(rosidl_default_generators REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(std_msgs REQUIRED)

  rosidl_auto_generate_interfaces()
endif()

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
if("$ENV{ROS_VERSION}" EQUAL 1)
set(_share_destination ${CATKIN_PACKAGE_SHARE_DESTINATION})
catkin_install_python(PROGRAMS src/tf_publish.py src/tf_set.py src/tf_reconfigure_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
elseif("$ENV{ROS_VERSION}" EQUAL 2)
  set(_share_destination share/${PROJECT_NAME})
  set(_scripts
    src/tf_publish.py
    src/tf_set.py
    src/tf_reconfigure_publisher.py
  )
  install(PROGRAMS ${_scripts}
    DESTINATION lib/${PROJECT_NAME}
  )

  if(BUILD_TESTING)
    find_package(launch_testing_ament_cmake REQUIRED)
    add_launch_test(test/tf_publish_launch_test.py)
  endif()
endif()

install(DIRECTORY samples
  DESTINATION ${_share_destination}
  PATTERN ".svn" EXCLUDE)

if("$ENV{ROS_VERSION}" EQUAL 1)
if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/tf_set.test)
endif()
elseif("$ENV{ROS_VERSION}" EQUAL 2)
  ament_package()
endif()
