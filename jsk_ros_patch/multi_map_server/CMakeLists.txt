# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(multi_map_server)
find_package(catkin REQUIRED COMPONENTS map_server rosconsole roscpp rospy nav_msgs tf)

find_package(PkgConfig)
pkg_check_modules(NEW_YAMLCPP yaml-cpp>=0.5)
if(NEW_YAMLCPP_FOUND)
add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NEW_YAMLCPP_FOUND)

catkin_package(
    DEPENDS rosconsole roscpp rospy nav_msgs tf
    CATKIN_DEPENDS
    INCLUDE_DIRS
    LIBRARIES
)

add_executable(multi_map_server src/main.cpp)
target_link_libraries(multi_map_server ${catkin_LIBRARIES} SDL SDL_image yaml-cpp)

install(TARGETS multi_map_server
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)



