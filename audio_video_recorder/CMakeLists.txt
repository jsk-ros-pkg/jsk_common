# Fall back to ROS1 when $ROS_VERSION is unset (pre-ros_environment distros).
if("$ENV{ROS_VERSION}" STREQUAL "")
  set(ENV{ROS_VERSION} 1)
endif()

if("$ENV{ROS_VERSION}" EQUAL 1)
cmake_minimum_required(VERSION 2.8.3)
elseif("$ENV{ROS_VERSION}" EQUAL 2)
  cmake_minimum_required(VERSION 3.8)
endif()

project(audio_video_recorder)

if("$ENV{ROS_VERSION}" EQUAL 1)
# jsk_ros1_ros2_compat's headers use C++11 `using Alias = Type;`
# declarations; indigo/kinetic's GCC (<6) defaults to pre-C++11 and needs
# this forced explicitly. Newer distros must NOT get this forced: their
# system headers (e.g. log4cxx's boost-std-configuration.h) require
# C++14/17 features from the compiler's own newer default, and an
# explicit -std=c++11 here would downgrade past that and break the build.
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 6.0)
  add_compile_options(-std=c++11)
endif()
find_package(catkin REQUIRED COMPONENTS roscpp audio_common_msgs message_filters sensor_msgs jsk_ros1_ros2_compat)
elseif("$ENV{ROS_VERSION}" EQUAL 2)
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(audio_common_msgs REQUIRED)
  find_package(message_filters REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(jsk_ros1_ros2_compat REQUIRED)
endif()

find_package(PkgConfig)
pkg_check_modules(GST1.0 gstreamer-1.0 REQUIRED)

find_package(Boost REQUIRED COMPONENTS thread)

if("$ENV{ROS_VERSION}" EQUAL 1)
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
  roscpp
  audio_common_msgs
  message_filters
  sensor_msgs
  jsk_ros1_ros2_compat
)
endif()

if("$ENV{ROS_VERSION}" EQUAL 1)
include_directories(include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${GST1.0_INCLUDE_DIRS}
)
elseif("$ENV{ROS_VERSION}" EQUAL 2)
  include_directories(include ${Boost_INCLUDE_DIRS} ${GST1.0_INCLUDE_DIRS})
endif()

add_executable(audio_video_recorder src/audio_video_recorder.cpp)
if("$ENV{ROS_VERSION}" EQUAL 1)
target_link_libraries(audio_video_recorder ${catkin_LIBRARIES} ${GST1.0_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(audio_video_recorder ${catkin_EXPORTED_TARGETS})
elseif("$ENV{ROS_VERSION}" EQUAL 2)
  ament_target_dependencies(audio_video_recorder rclcpp audio_common_msgs message_filters sensor_msgs jsk_ros1_ros2_compat)
  target_link_libraries(audio_video_recorder ${GST1.0_LIBRARIES} ${Boost_LIBRARIES})
endif()

if("$ENV{ROS_VERSION}" EQUAL 1)
install(TARGETS audio_video_recorder
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch sample
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
return()
endif()

install(TARGETS audio_video_recorder DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch sample DESTINATION share/${PROJECT_NAME})
ament_package()
