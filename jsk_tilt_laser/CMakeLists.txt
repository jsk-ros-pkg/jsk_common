cmake_minimum_required(VERSION 2.8.3)
project(jsk_tilt_laser)

# we do not require laser_assembler on travis
# because rosdep does not support it.
if(NOT $ENV{ROS_DISTRO} STREQUAL "groovy")
  set(laser_assembler "laser_assembler")
endif(NOT $ENV{ROS_DISTRO} STREQUAL "groovy")

find_package(multisense_lib QUIET)
if(multisense_lib_FOUND)
  message(STATUS "Found multisense_lib")
  set(multisense_lib multisense_lib)
endif(multisense_lib_FOUND)

find_package(catkin REQUIRED
  dynamic_reconfigure sensor_msgs ${laser_assembler}
  tf_conversions tf cmake_modules
  ${multisense_lib}
  )


find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

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

if(multisense_lib_FOUND)
  add_executable(multisense_killer src/multisense_killer.cpp)
  target_link_libraries(multisense_killer ${catkin_LIBRARIES})
  install(TARGETS multisense_killer
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
else(multisense_lib_FOUND)
  message(STATUS "multisense_killer is not build, because of multisense_lib is not found")
endif(multisense_lib_FOUND)

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
install(DIRECTORY cfg config launch scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
