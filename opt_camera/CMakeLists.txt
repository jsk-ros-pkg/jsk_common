cmake_minimum_required(VERSION 2.8.3)
project(opt_camera)

set(${PROJECT_NAME}_CATKIN_DEPS
  rospack sensor_msgs driver_base image_proc dynamic_reconfigure camera_calibration_parsers compressed_image_transport image_proc compressed_image_transport cv_bridge
)
find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_DEPS})

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)

# Dynamic reconfigure support
generate_dynamic_reconfigure_options(cfg/OptNM33Camera.cfg)

catkin_package(
    CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS}
    LIBRARIES #
    INCLUDE_DIRS #
    DEPENDS #
)

find_package(OpenCV REQUIRED)

include_directories(${catkin_INCLUDE_DIRS}  ${OpenCV_INCLUDE_DIRS})

add_executable(nm33_node src/nm33_node.cpp src/opt_nm33_uvc/opt_nm33_camera.cpp src/opt_nm33_uvc/opt_nm33_uvc.cpp)
add_executable(init_xu_register src/opt_nm33_uvc/init_xu_register.cpp src/opt_nm33_uvc/opt_nm33_uvc.cpp)
target_link_libraries(nm33_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(nm33_node_non_std_uvc src/nm33_node.cpp src/opt_nm33_uvc/opt_nm33_camera.cpp src/opt_nm33_uvc/opt_nm33_uvc.cpp)
add_executable(init_xu_register_non_std_uvc src/opt_nm33_uvc/init_xu_register.cpp src/opt_nm33_uvc/opt_nm33_uvc.cpp)
target_link_libraries(nm33_node_non_std_uvc ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_dependencies(nm33_node ${PROJECT_NAME}_gencfg)
add_dependencies(nm33_node_non_std_uvc ${PROJECT_NAME}_gencfg)

install(TARGETS nm33_node nm33_node_non_std_uvc init_xu_register init_xu_register_non_std_uvc
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)



