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
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_link_boost(rosping thread)
rosbuild_add_executable(rosping src/rosping.cpp src/ping.cpp)
#target_link_libraries(example ${PROJECT_NAME})
rosbuild_link_boost(rosping system)

string(ASCII 27 ESCAPE)
add_custom_command(
  OUTPUT message
  COMMAND echo "${ESCAPE}[34m#\ type\ following\ command\ before\ execute\ rosping"
  COMMAND echo "sudo\ chown\ root.root\ ./bin/rosping\;\ sudo\ chmod 4755\ ./bin/rosping${ESCAPE}[0m"
  DEPENDS bin/rosping)
add_custom_target(message_all ALL DEPENDS message)


