cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_find_ros_package(actionlib)
include(${actionlib_PACKAGE_PATH}/cmake/actionbuild.cmake)
genaction()
rosbuild_init()
rosbuild_genmsg()
include_directories(include)


