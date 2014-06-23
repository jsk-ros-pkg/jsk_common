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
set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

# Dynamic reconfigure support
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

find_package(OpenCV REQUIRED)
#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
rosbuild_add_executable(nm33_node src/nm33_node.cpp src/opt_nm33_uvc/opt_nm33_camera.cpp src/opt_nm33_uvc/opt_nm33_uvc.cpp)
rosbuild_add_executable(init_xu_register src/opt_nm33_uvc/init_xu_register.cpp src/opt_nm33_uvc/opt_nm33_uvc.cpp)
target_link_libraries(nm33_node rospack ${OpenCV_LIBRARIES})

#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_executable(nm33_node_non_std_uvc src/nm33_node.cpp src/opt_nm33_uvc/opt_nm33_camera.cpp src/opt_nm33_uvc/opt_nm33_uvc.cpp)
target_link_libraries(nm33_node_non_std_uvc rospack ${OpenCV_LIBRARIES})
rosbuild_add_executable(init_xu_register_non_std_uvc src/opt_nm33_uvc/init_xu_register.cpp src/opt_nm33_uvc/opt_nm33_uvc.cpp)
rosbuild_add_compile_flags(nm33_node_non_std_uvc -DNON_STD_UVC=1)
rosbuild_add_compile_flags(init_xu_register_non_std_uvc -DNON_STD_UVC=1)
