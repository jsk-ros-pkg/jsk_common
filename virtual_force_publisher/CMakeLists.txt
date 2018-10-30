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

# From https://github.com/jsk-ros-pkg/jsk_recognition/pull/2345
# Install header files directly into ${CATKIN_PACKAGE_INCLUDE_DESTINATION}.
# If the package has setup.py and modules under src/${PROJECT_NAME}/,
# install python executables directly into ${CATKIN_PACKAGE_BIN_DESTINATION}.
# However, if it doesn't have setup.py, directories including python executables
# should be installed recursively into ${CATKIN_PACKAGE_SHARE_DESTINATION}.
# Also, other directories like 'launch' or 'sample' must be installed
# recursively into ${CATKIN_PACKAGE_SHARE_DESTINATION}.
# Be careful that 'launch' and 'launch/' are different: the former is directory
# and the latter is each content.
install(TARGETS virtual_force_publisher DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
