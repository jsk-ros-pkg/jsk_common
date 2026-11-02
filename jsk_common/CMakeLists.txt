# $ENV{ROS_VERSION} is set by ros_environment, which is missing on
# pre-ros_environment distros (e.g. indigo) and on ROS2 builds that do not
# depend on it (e.g. the buildfarm's binary jobs). Guess it then: ament sets
# AMENT_PREFIX_PATH (bloom's debian/rules passes it to cmake, setup.sh and
# colcon set the environment variable), catkin never does.
if("$ENV{ROS_VERSION}" STREQUAL "")
  if(DEFINED AMENT_PREFIX_PATH OR DEFINED ENV{AMENT_PREFIX_PATH})
    set(ENV{ROS_VERSION} 2)
  else()
    set(ENV{ROS_VERSION} 1)
  endif()
  message(WARNING "ROS_VERSION is not set, assuming ROS_VERSION=$ENV{ROS_VERSION}")
endif()

# cmake_minimum_required() must run before project(), so it needs its own
# ROS_VERSION branch here (indigo's cmake is too old for VERSION 3.8).
if("$ENV{ROS_VERSION}" EQUAL 1)
cmake_minimum_required(VERSION 2.8.3)
elseif("$ENV{ROS_VERSION}" EQUAL 2)
  cmake_minimum_required(VERSION 3.8)
endif()
project(jsk_common)

if("$ENV{ROS_VERSION}" EQUAL 1)
find_package(catkin REQUIRED)
catkin_package()
elseif("$ENV{ROS_VERSION}" EQUAL 2)
  find_package(ament_cmake REQUIRED)
  ament_package()
endif()
