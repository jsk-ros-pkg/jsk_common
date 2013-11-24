# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(rosping)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)


#common commands for building c++ executables and libraries
#add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#
# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_add_boost_directories()
find_package(Boost REQUIRED COMPONENTS system)
include_directories(${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
#target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})
#target_link_libraries(rosping ${Boost_LIBRARIES})
add_executable(${PROJECT_NAME} src/rosping.cpp src/ping.cpp)
#target_link_libraries(example ${PROJECT_NAME})
target_link_libraries(rosping ${Boost_LIBRARIES} ${catkin_LIBRARIES})

string(ASCII 27 ESCAPE)
message("${ESCAPE}[34m
# type following command before execute rosping
sudo chown root.root ./bin/rosping
sudo chmod 4755 ./bin/rosping
${ESCAPE}[0m")


catkin_package(
    DEPENDS roscpp std_msgs
    CATKIN-DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)
