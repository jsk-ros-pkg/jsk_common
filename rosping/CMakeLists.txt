# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(rosping)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs rostest)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

catkin_package(
    DEPENDS
    CATKIN_DEPENDS roscpp std_msgs
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
  COMMAND sudo -n sh -c 'cd ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}\; chown root.root rosping\; chmod 4755 rosping' || (echo "${ESCAPE}[34m#\ type\ following\ command\ before\ execute\ rosping...\ sudo\ chown\ root.root\ ./bin/rosping\;\ sudo\ chmod 4755\ ./bin/rosping${ESCAPE}[0m")
  DEPENDS bin/rosping)
add_custom_target(message_all ALL DEPENDS message)

install(
  PROGRAMS ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/rosping
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE SETUID
)
install(CODE
  "execute_process(COMMAND sudo -n sh -c \"cd \$ENV{DESTDIR}/${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION} ; chown root.root rosping; ls -al rosping; chmod 4755 rosping\")
")

install(DIRECTORY test
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  )
if (NOT $ENV{ROS_DISTRO} STREQUAL "indigo")
  add_rostest(test/test-rosping.test)
endif(NOT $ENV{ROS_DISTRO} STREQUAL "indigo")
