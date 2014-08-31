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
include(${PROJECT_SOURCE_DIR}/cmake/nodelet.cmake)
macro(jsk_topic_tools_nodelet _nodelet_cpp _nodelet_class _single_nodelet_exec_name)
  jsk_nodelet(${_nodelet_cpp} ${_nodelet_class} ${_single_nodelet_exec_name}
    jsk_topic_tools_nodelet_sources)
endmacro(jsk_topic_tools_nodelet _nodelet_cpp _nodelet_class _single_nodelet_exec_name)

jsk_topic_tools_nodelet(src/lightweight_throttle_nodelet.cpp
  "jsk_topic_tools/LightweightThrottle" "lightweight_throttle")
jsk_topic_tools_nodelet(src/mux_nodelet.cpp
  "jsk_topic_tools/MUX" "mux")
jsk_topic_tools_nodelet(src/relay_nodelet.cpp
  "jsk_topic_tools/Relay" "relay")
jsk_topic_tools_nodelet(src/block_nodelet.cpp
  "jsk_topic_tools/Block" "block")
jsk_topic_tools_nodelet(src/hz_measure_nodelet.cpp
  "jsk_topic_tools/HzMeasure" "hz_measure")


rosbuild_add_library (jsk_topic_tools
  ${jsk_topic_tools_nodelet_sources}
  src/rosparam_utils.cpp
  src/vital_checker.cpp
  src/time_accumulator.cpp)

#target_link_libraries(example ${PROJECT_NAME})
rosbuild_add_rostest(test/test_topic_buffer.test)
rosbuild_add_rostest(test/test_topic_buffer_fixed_rate.test)
rosbuild_add_rostest(test/test_topic_buffer_update_rate.test)
rosbuild_add_rostest(test/test_topic_buffer_fixed_rate_and_update_rate.test)
rosbuild_add_rostest(test/test_topic_buffer_close_wait.test)
rosbuild_add_rostest(test/test_lightweight_throttle.test)
rosbuild_add_rostest(test/test_topic_compare.test)
rosbuild_add_rostest(test/test_hz_measure.test)
rosbuild_add_rostest(test/test_block.test)
