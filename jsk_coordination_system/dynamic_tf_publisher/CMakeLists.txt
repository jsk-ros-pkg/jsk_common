cmake_minimum_required(VERSION 2.8.3)
project(dynamic_tf_publisher)

find_package(catkin REQUIRED
  COMPONENTS
  cmake_modules
  dynamic_reconfigure
  geometry_msgs
  interactive_markers
  message_generation
  roscpp
  rospy
  std_msgs
  tf2
  tf2_ros
  visualization_msgs
)
add_service_files(DIRECTORY srv
  FILES DeleteTF.srv DissocTF.srv SetDynamicTF.srv AssocTF.srv)
generate_messages(
  DEPENDENCIES geometry_msgs
)
generate_dynamic_reconfigure_options(
  cfg/TfParameter.cfg
)

catkin_package(
  DEPENDS
  CATKIN_DEPENDS
  geometry_msgs
  std_msgs
  visualization_msgs
  message_runtime
  INCLUDE_DIRS include
  LIBRARIES
)

add_executable(${PROJECT_NAME} src/dynamic_tf_publisher.cpp)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_gencpp)
include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

# ------------------------------------------------------------------------------------
# Install
# ------------------------------------------------------------------------------------

install(TARGETS
  # Libraries
  ${PROJECT_NAME}
  # Node executables
  ${${PROJECT_NAME}_node_executables}
  # Nodelet executables
  ${${PROJECT_NAME}_nodelet_executables}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY scripts/
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        USE_SOURCE_PERMISSIONS
)

install(DIRECTORY samples
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
