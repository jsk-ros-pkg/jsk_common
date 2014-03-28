# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(stereo_synchronizer)

find_package(catkin REQUIRED COMPONENTS sensor_msgs image_transport)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

add_executable(stereo_synchronizer src/stereo_synchronizer.cpp)
target_link_libraries(stereo_synchronizer ${catkin_LIBRARIES})
add_executable(cr_synchronizer src/cr_synchronizer.cpp)
target_link_libraries(cr_synchronizer ${catkin_LIBRARIES})

catkin_package(
    DEPENDS sensor_msgs image_transport
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

install(TARGETS stereo_synchronizer cr_synchronizer
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES stereo_test.launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)



