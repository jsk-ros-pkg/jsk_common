cmake_minimum_required(VERSION 2.8.3)
project(tf_relay)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    tf2
    tf2_ros
    geometry_msgs
)

find_package(Boost REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES tf_relay
  CATKIN_DEPENDS
    roscpp
    tf2
    tf2_ros
    geometry_msgs
  DEPENDS
    Boost
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(tf_relay
  src/tf_relay_node.cpp
  src/tf_relay.cpp
)
target_link_libraries(tf_relay
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)
