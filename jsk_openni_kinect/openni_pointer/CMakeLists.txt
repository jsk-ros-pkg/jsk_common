if(NOT ("$ENV{ROS_Distributions}" STREQUAL "electric" OR "$ENV{ROS_DISTRO}" STREQUAL "fuerte"))
  include(catkin.cmake)
  return()
endif()
cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
rosbuild_add_executable(openni_pointer src/openni_pointer.cpp)

# https://code.ros.org/trac/ros-pkg/ticket/5287
target_link_libraries(openni_pointer XnVNite)
