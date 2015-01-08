cmake_minimum_required(VERSION 2.8.3)
project(mini_maxwell)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules roslib dynamic_reconfigure)

generate_dynamic_reconfigure_options(
  cfg/RosClient.cfg
  cfg/DRCEnvironment.cfg
)

#include_directories(${Boost_INCLUDE_DIRS})

# nodelet shared object
include_directories(${catkin_INCLUDE_DIRS} include)

# generate_messages()

catkin_package(
    DEPENDS
    CATKIN_DEPENDS dynamic_reconfigure
)

install(DIRECTORY scripts launch
  DESTINATION
  ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  )
