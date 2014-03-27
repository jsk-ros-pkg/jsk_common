if(NOT USE_ROSBUILD)
  include(catkin.cmake)
  return()
endif()
cmake_minimum_required(VERSION 2.4.6 FATAL_ERROR)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()

# if not found map_server
rosbuild_find_ros_package(jsk_tools)
include(${jsk_tools_PACKAGE_PATH}/cmake/download_package.cmake)
download_package_for_groovy(map_server http://github.com/ros-planning/navigation groovy)

rosbuild_add_executable(bin/map_server src/main.cpp)
target_link_libraries(bin/map_server image_loader SDL SDL_image yaml-cpp)
