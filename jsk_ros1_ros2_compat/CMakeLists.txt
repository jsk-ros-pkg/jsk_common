# Fall back to ROS1 when $ENV{ROS_VERSION} is unset (pre-ros_environment
# distros); every ROS2 environment always sets it via ros_environment.
if("$ENV{ROS_VERSION}" STREQUAL "")
  set(ENV{ROS_VERSION} 1)
endif()

# cmake_minimum_required() must run before project(), so it needs its own
# ROS_VERSION branch here.
if("$ENV{ROS_VERSION}" EQUAL 1)
  cmake_minimum_required(VERSION 2.8.3)
elseif("$ENV{ROS_VERSION}" EQUAL 2)
  cmake_minimum_required(VERSION 3.8)
endif()

project(jsk_ros1_ros2_compat)

if("$ENV{ROS_VERSION}" EQUAL 1)
  find_package(catkin REQUIRED COMPONENTS roscpp)
  catkin_python_setup()
  catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp
  )
elseif("$ENV{ROS_VERSION}" EQUAL 2)
  find_package(ament_cmake REQUIRED)
  find_package(ament_cmake_python REQUIRED)
  find_package(rclcpp REQUIRED)

  ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR src/${PROJECT_NAME})
endif()

if("$ENV{ROS_VERSION}" EQUAL 1)
  include_directories(include ${catkin_INCLUDE_DIRS})

  install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )
  return()
endif()

install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
)
ament_export_include_directories(include)
ament_export_dependencies(rclcpp)
ament_package()
