cmake_minimum_required(VERSION 2.4.6)

project(virtual_force_publisher)

find_package(catkin REQUIRED COMPONENTS geometry_msgs sensor_msgs kdl_parser urdf tf_conversions)

catkin_package(
    DEPENDS
    CATKIN_DEPENDS geometry_msgs sensor_msgs kdl_parser urdf tf_conversions
    INCLUDE_DIRS
    LIBRARIES
)

include_directories(${catkin_INCLUDE_DIRS})
add_executable(virtual_force_publisher src/virtual_force_publisher.cpp)
target_link_libraries(virtual_force_publisher ${catkin_LIBRARIES})


install(TARGETS virtual_force_publisher DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


