#if(NOT ("$ENV{ROS_Distributions}" STREQUAL "electric" OR "$ENV{ROS_DISTRO}" STREQUAL "fuerte"))
#  include(catkin.cmake)
#  return()
#endif()
cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()

find_package(OpenCV REQUIRED)
rosbuild_add_executable(feature0d_view src/feature0d_view.cpp)
target_link_libraries(feature0d_view ${OpenCV_LIBRARIES})

rosbuild_genmsg()
rosbuild_gensrv()
