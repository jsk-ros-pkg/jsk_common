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



#
# Install
#
install(TARGETS tf_relay
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
       )

install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)



#
# Testgin
#
if (CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    add_rostest(test/test_tf_relay.test)
endif()
