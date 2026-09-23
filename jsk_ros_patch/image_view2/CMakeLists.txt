# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
# Fall back to ROS1 when $ROS_VERSION is unset (pre-ros_environment distros,
# e.g. indigo); every ROS2 environment always sets it via ros_environment.
if("$ENV{ROS_VERSION}" STREQUAL "")
  set(ENV{ROS_VERSION} 1)
endif()

# cmake_minimum_required() must run before project(), so it needs its own
# ROS_VERSION branch here (indigo's cmake is too old for VERSION 3.8).
if("$ENV{ROS_VERSION}" EQUAL 1)
cmake_minimum_required(VERSION 2.8.3)
elseif("$ENV{ROS_VERSION}" EQUAL 2)
  cmake_minimum_required(VERSION 3.8)
endif()
project(image_view2)

###############################################################################
# find_package()
###############################################################################
if("$ENV{ROS_VERSION}" EQUAL 1)
# jsk_ros1_ros2_compat's headers use C++11 `using Alias = Type;`
# declarations; indigo/kinetic's GCC (<6) defaults to pre-C++11 and needs
# this forced explicitly. Newer distros must NOT get this forced: their
# system headers (e.g. log4cxx's boost-std-configuration.h) require
# C++14/17 features from the compiler's own newer default, and an
# explicit -std=c++11 here would downgrade past that and break the build.
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 6.0)
  add_compile_options(-std=c++11)
endif()
# Use ccache if installed to make it fast to generate object files
if (CMAKE_VERSION VERSION_LESS 3.4)
  find_program(CCACHE_FOUND ccache)
  if(CCACHE_FOUND)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
  endif(CCACHE_FOUND)
endif()

find_package(catkin REQUIRED COMPONENTS roscpp dynamic_reconfigure cv_bridge std_msgs sensor_msgs geometry_msgs image_transport tf2_ros tf2_geometry_msgs image_geometry message_filters message_generation std_srvs pcl_ros jsk_ros1_ros2_compat)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS)
find_package(PCL REQUIRED)

elseif("$ENV{ROS_VERSION}" EQUAL 2)
  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
  endif()

  find_package(ament_cmake REQUIRED)
  find_package(rosidl_default_generators REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(cv_bridge REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(image_geometry REQUIRED)
  find_package(image_transport REQUIRED)
  find_package(message_filters REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(std_srvs REQUIRED)
  find_package(tf2_ros REQUIRED)
  find_package(tf2_geometry_msgs REQUIRED)
  find_package(jsk_ros1_ros2_compat REQUIRED)
  # boost::mutex/boost::thread (image_view2.h's *_mutex_ members).
  find_package(Boost REQUIRED COMPONENTS thread)
  find_package(OpenCV REQUIRED)
  # publishPointArray() only needs bare PCL types + pcl_conversions (not
  # pcl_ros, which stays ROS1-only -- see points_rectangle_extractor
  # note above).
  find_package(PCL REQUIRED COMPONENTS common)
  find_package(pcl_conversions REQUIRED)
endif()

###############################################################################
# message/interface generation
###############################################################################
if("$ENV{ROS_VERSION}" EQUAL 1)
generate_dynamic_reconfigure_options(
  cfg/ImageView2.cfg
)
add_message_files(
  FILES ImageMarker2.msg PointArrayStamped.msg MouseEvent.msg
)
add_service_files(
  FILES ChangeMode.srv
  )
generate_messages(DEPENDENCIES geometry_msgs std_msgs)

set(_extra_include_dirs ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

catkin_package(
  DEPENDS OpenCV PCL
  CATKIN_DEPENDS roscpp cv_bridge std_msgs sensor_msgs geometry_msgs image_transport tf2_ros tf2_geometry_msgs image_geometry message_filters jsk_ros1_ros2_compat
  INCLUDE_DIRS # TODO include
  LIBRARIES # TODO
)

set(_share_destination ${CATKIN_PACKAGE_SHARE_DESTINATION})

elseif("$ENV{ROS_VERSION}" EQUAL 2)
  # rosidl_auto_generate_interfaces() (called in the build-targets section
  # below, once the image_view2 target it links against exists) discovers
  # msg/*.msg and srv/*.srv on its own and derives DEPENDENCIES from
  # package.xml's <build_depend> tags -- nothing to do here.
  set(_share_destination share/${PROJECT_NAME})
  set(_extra_include_dirs ${PCL_INCLUDE_DIRS})
endif()

###############################################################################
# build targets
###############################################################################
# NOTE: points_rectangle_extractor (PCL/message_filters-based) is a
# stretch-goal not yet ROS2-ported (see package.xml); only image_view2
# itself is built under ROS2.
#
# Under ROS2 the executable CMake *target* can't be named "image_view2"
# (rosidl_auto_generate_interfaces() below already creates a custom
# target literally called "${PROJECT_NAME}" for message generation, and
# CMake target names must be unique) -- so it's "image_view2_node"
# there, with OUTPUT_NAME set back to "image_view2" so the installed
# binary is still invoked the same way (`ros2 run image_view2
# image_view2`) as under ROS1.
if("$ENV{ROS_VERSION}" EQUAL 1)
set(_image_view2_target image_view2)
elseif("$ENV{ROS_VERSION}" EQUAL 2)
  set(_image_view2_target image_view2_node)
endif()
add_executable(${_image_view2_target} image_view2.cpp image_view2_node.cpp)
target_include_directories(${_image_view2_target} PUBLIC include ${_extra_include_dirs})

if("$ENV{ROS_VERSION}" EQUAL 1)
target_link_libraries(image_view2 ${OpenCV_LIBS} ${catkin_LIBRARIES})
add_dependencies(image_view2 ${PROJECT_NAME}_gencpp ${PROJECT_NAME}_gencfg)

add_executable(points_rectangle_extractor points_rectangle_extractor.cpp)
target_include_directories(points_rectangle_extractor PUBLIC ${_extra_include_dirs})
target_link_libraries(points_rectangle_extractor ${Boost_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(points_rectangle_extractor ${PROJECT_NAME}_gencpp)

elseif("$ENV{ROS_VERSION}" EQUAL 2)
  set_target_properties(${_image_view2_target} PROPERTIES OUTPUT_NAME image_view2)
  ament_target_dependencies(${_image_view2_target}
    rclcpp
    cv_bridge
    geometry_msgs
    image_geometry
    image_transport
    message_filters
    sensor_msgs
    std_msgs
    std_srvs
    tf2_ros
    tf2_geometry_msgs
    pcl_conversions
    jsk_ros1_ros2_compat
  )
  target_link_libraries(${_image_view2_target} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${Boost_LIBRARIES})

  # Not rosidl_auto_generate_interfaces() (which only globs msg/*.msg):
  # ImageMarker2.msg needs a ROS2-only variant (ros2/msg/ImageMarker2.msg,
  # see that file for why), so the interface file list is given
  # explicitly here instead.
  rosidl_generate_interfaces(${PROJECT_NAME}
    "ros2/msg/ImageMarker2.msg"
    "msg/MouseEvent.msg"
    "msg/PointArrayStamped.msg"
    "srv/ChangeMode.srv"
    DEPENDENCIES geometry_msgs std_msgs
  )
  rosidl_get_typesupport_target(_image_view2_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
  target_link_libraries(${_image_view2_target} ${_image_view2_typesupport_target})

  if(BUILD_TESTING)
    find_package(launch_testing_ament_cmake REQUIRED)
    add_launch_test(test/rectangle_mouse_event_launch_test.py)
  endif()
endif()

###############################################################################
# install
###############################################################################
# From https://github.com/jsk-ros-pkg/jsk_recognition/pull/2345
install(DIRECTORY test
  DESTINATION ${_share_destination}
  USE_SOURCE_PERMISSIONS
)

if("$ENV{ROS_VERSION}" EQUAL 1)
install(TARGETS image_view2 points_rectangle_extractor
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  execute_process(
    COMMAND python -c "import sys; sys.exit(ord('$ENV{ROS_DISTRO}'[0]) >= ord('indigo'[0]))"
    RESULT_VARIABLE DISTRO_GE_INDIGO
  )
  if(${DISTRO_GE_INDIGO})
    # FIXME: jsk_tools/test_topic_published.py does not work on hydro travis/jenkins
    # https://github.com/jsk-ros-pkg/jsk_common/pull/1293#issuecomment-164158260
    add_rostest(test/rectangle_mouse_event.launch)
  endif()
endif()

elseif("$ENV{ROS_VERSION}" EQUAL 2)
  install(TARGETS ${_image_view2_target}
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
  ament_package()
endif()
