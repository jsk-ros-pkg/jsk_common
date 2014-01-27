if(NOT USE_ROSBUILD)
  include(catkin.cmake)
  return()
endif()
cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

rosbuild_genmsg()

# Image viewers
find_package(OpenCV REQUIRED)

rosbuild_add_executable(image_view2 image_view2.cpp)
target_link_libraries(image_view2 ${OpenCV_LIBS})

rosbuild_add_boost_directories()
rosbuild_add_executable(points_rectangle_extractor points_rectangle_extractor.cpp)
rosbuild_link_boost(points_rectangle_extractor signals)