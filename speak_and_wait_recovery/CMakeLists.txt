cmake_minimum_required(VERSION 3.0.2)
project(speak_and_wait_recovery)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
    COMPONENTS
        costmap_2d
        nav_core
        pluginlib
        roscpp
        tf2
        tf2_ros
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES speak_and_wait_recovery
  CATKIN_DEPENDS
        costmap_2d
        nav_core
        pluginlib
        roscpp
        tf2
        tf2_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(speak_and_wait_recovery src/speak_and_wait_recovery.cpp)
add_dependencies(speak_and_wait_recovery ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(speak_and_wait_recovery ${catkin_LIBRARIES})

install(TARGETS speak_and_wait_recovery
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
       )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(FILES speak_and_wait_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

