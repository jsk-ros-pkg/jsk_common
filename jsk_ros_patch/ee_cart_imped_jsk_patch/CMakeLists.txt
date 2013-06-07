cmake_minimum_required(VERSION 2.4.6)

set(package "ee_cart_imped")
execute_process(COMMAND rosstack find ee_cart_imped OUTPUT_VARIABLE ${package}_PACKAGE_PATH OUTPUT_STRIP_TRAILING_WHITESPACE)
message("ee_cart_imped stack path -> ${${package}_PACKAGE_PATH}")

set(PATCH_BATCH_FILE ${PROJECT_SOURCE_DIR}/${package}_patch.bat )
file(WRITE ${PATCH_BATCH_FILE} "set -x\npatch -p0 -i ${PROJECT_SOURCE_DIR}/patches/${package}.patch")
execute_process( COMMAND cmake -E chdir ${${package}_PACKAGE_PATH} sh ${PATCH_BATCH_FILE} RESULT_VARIABLE RESULT_PATCH )
message( STATUS "Patch result: ${RESULT_PATCH}, SOURCE DIR: ${${package}_PACKAGE_PATH}" )
file(REMOVE ${PATCH_BATCH_FILE})


#include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

#rosbuild_init()

#set the default path for built executables to the "bin" directory
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
