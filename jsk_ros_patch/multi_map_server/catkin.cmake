# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(multi_map_server)
find_package(catkin REQUIRED COMPONENTS rosconsole roscpp rospy nav_msgs tf)

# if not found map_server
find_package(map_server QUIET)
if(NOT map_server_FOUND)
  if(EXISTS /opt/ros/$ENV{ROS_DISTRO}/stacks/navigation/map_server)
    set(map_server_PREFIX /opt/ros/$ENV{ROS_DISTRO}/stacks/navigation/map_server)
    include_directories(${map_server_PREFIX}/include)
  endif()
endif()
if(NOT map_server_PREFIX)
  link_directories(${map_server_PREFIX}/lib)
  find_package(jsk_tools)
  download_package_for_groovy(map_server http://github.com/ros-planning/navigation groovy)
endif()

find_package(PkgConfig)
pkg_check_modules(NEW_YAMLCPP yaml-cpp>=0.5)
if(NEW_YAMLCPP_FOUND)
add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NEW_YAMLCPP_FOUND)

link_directories(${map_server_PREFIX}/lib)
add_executable(map_server src/main.cpp)
target_link_libraries(map_server ${catkin_LIBRARIES} image_loader SDL SDL_image yaml-cpp)

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    DEPENDS rosconsole roscpp rospy nav_msgs tf
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

install(TARGETS map_server
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)



