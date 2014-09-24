cmake_minimum_required(VERSION 2.8.3)
project(laser_filters_jsk_patch)

find_package(catkin REQUIRED COMPONENTS laser_filters filters)
catkin_package(CATKIN_DEPENDS laser_filters filters)
include_directories(include ${catkin_INCLUDE_DIRS})
add_library(laser_scan_filters_jsk_patch SHARED src/laser_scan_filters.cpp)
target_link_libraries(laser_scan_filters_jsk_patch ${catkin_LIBRARIES})

install(TARGETS laser_scan_filters_jsk_patch
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
