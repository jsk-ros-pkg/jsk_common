if(NOT USE_ROSBUILD)
  include(catkin.cmake)
  return()
endif()
cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
rosbuild_add_executable(topic_buffer_server src/topic_buffer_server.cpp)
rosbuild_add_executable(topic_buffer_client src/topic_buffer_client.cpp)
rosbuild_add_executable(specific_transform_publisher src/specific_transform_publisher.cpp)
rosbuild_add_executable(specific_transform_subscriber src/specific_transform_subscriber.cpp)
rosbuild_add_executable(transform_merger src/transform_merger.cpp)

rosbuild_add_library (jsk_topic_tools
  src/lightweight_throttle_nodelet.cpp
  src/mux_nodelet.cpp
  )

#target_link_libraries(example ${PROJECT_NAME})
rosbuild_add_rostest(test/test_topic_buffer.test)
rosbuild_add_rostest(test/test_topic_buffer_fixed_rate.test)
rosbuild_add_rostest(test/test_topic_buffer_update_rate.test)
rosbuild_add_rostest(test/test_topic_buffer_fixed_rate_and_update_rate.test)
rosbuild_add_rostest(test/test_topic_buffer_close_wait.test)
rosbuild_add_rostest(test/test_lightweight_throttle.test)
rosbuild_add_rostest(test/test_topic_compare.test)