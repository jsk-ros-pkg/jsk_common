if(NOT USE_ROSBUILD)
  include(catkin.cmake)
  return()
endif()
cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()

rosbuild_genmsg()
rosbuild_gensrv()

find_package(OpenCV REQUIRED)

rosbuild_add_executable(feature0d_view src/feature0d_view.cpp)
target_link_libraries(feature0d_view ${OpenCV_LIBRARIES})
add_dependencies(feature0d_view ${PROJECT_NAME}_generate_messages_cpp)

