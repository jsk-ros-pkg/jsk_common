cmake_minimum_required(VERSION 2.8.3)
project(jsk_creative_gesture_launch)

find_package(catkin REQUIRED COMPONENTS
  pcl_msgs
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  )

include_directories(/opt/softkinetic/DepthSenseSDK/include)
link_directories(DEPTH_SENSE_LIBRARIES DepthSense /opt/softkinetic/DepthSenseSDK/lib)
## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system thread)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES jsk_creative_gesture_ros
  CATKIN_DEPENDS pcl_msgs pcl_ros roscpp rospy sensor_msgs std_msgs
  )

###########
## Build ##
###########

if ( EXISTS /opt/softkinect/DepthSenseSDK )
  include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    )

  ## Declare a cpp library
  add_library(jsk_creative_gesture_ros
    SHARED
    src/jsk_creative_gesture_ros.cpp
    )
  target_link_libraries(jsk_creative_gesture_ros ${catkin_LIBRARIES} ${pcl_ros_LIBRARIES} DepthSense)
  add_dependencies(jsk_creative_gesture_ros ${PROJECT_NAME}_gencpp ${PROJECT_NAME}_gencfg)


  ## Declare a cpp executable
  add_executable(open_creative_gesture src/open_creative_gesture.cpp)

  ## Specify libraries to link a library or executable target against
  target_link_libraries(open_creative_gesture
    ${catkin_LIBRARIES}
    jsk_creative_gesture_ros
    )
  add_dependencies(open_creative_gesture jsk_creative_gesture_launch_generate_messages_cpp)
endif( EXISTS /opt/softkinect/DepthSenseSDK)
#############
## Install ##
#############