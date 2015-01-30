cmake_minimum_required(VERSION 2.8.3)
project(jsk_tilt_laser)

# we do not require laser_assembler on travis
# because rosdep does not support it.
if(NOT $ENV{ROS_DISTRO} STREQUAL "groovy")
  set(laser_assembler "laser_assembler")
endif(NOT $ENV{ROS_DISTRO} STREQUAL "groovy")

find_package(catkin REQUIRED
  dynamic_reconfigure sensor_msgs ${laser_assembler}
  tf_conversions tf cmake_modules
  )

find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

generate_dynamic_reconfigure_options(
  cfg/DynamixelTiltController.cfg
  )


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES moge
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)
if(NOT $ENV{ROS_DISTRO} STREQUAL "groovy")
  add_executable(spin_laser_snapshotter src/spin_laser_snapshotter.cpp)
  target_link_libraries(spin_laser_snapshotter ${catkin_LIBRARIES})
  install(TARGETS spin_laser_snapshotter
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
endif(NOT $ENV{ROS_DISTRO} STREQUAL "groovy")