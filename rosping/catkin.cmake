# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(rosping)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

catkin_package(
    DEPENDS
    CATKIN-DEPENDS roscpp std_msgs
    INCLUDE_DIRS
    LIBRARIES
)

find_package(Boost REQUIRED COMPONENTS system)
include_directories(${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
add_executable(rosping src/rosping.cpp src/ping.cpp)
target_link_libraries(rosping ${Boost_LIBRARIES} ${catkin_LIBRARIES})

string(ASCII 27 ESCAPE)
add_custom_command(
  OUTPUT message
  COMMAND echo "${ESCAPE}[34m#\ type\ following\ command\ before\ execute\ rosping"
  COMMAND echo "sudo\ chown\ root.root\ ./bin/rosping\;\ sudo\ chmod 4755\ ./bin/rosping${ESCAPE}[0m"
  DEPENDS bin/rosping)
add_custom_target(message_all ALL DEPENDS message)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

