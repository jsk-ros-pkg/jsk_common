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
    CFG_EXTRAS ${PROJECT_NAME}-extras.cmake
    CATKIN_DEPENDS roscpp
  )
elseif("$ENV{ROS_VERSION}" EQUAL 2)
  find_package(ament_cmake REQUIRED)
  find_package(ament_cmake_python REQUIRED)
  find_package(rclcpp REQUIRED)

  ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR src/${PROJECT_NAME})

  # uv for catkin_uv.cmake's jsk_ros1_ros2_compat_generate_uv_venv(): there
  # is no rosdep key (nor Ubuntu package) for uv, so install a pinned
  # prebuilt binary next to this package with uv's own installer.
  set(JSK_ROS1_ROS2_COMPAT_UV_VERSION "0.11.28")
  set(_uv_dir ${CMAKE_CURRENT_BINARY_DIR}/uv-${JSK_ROS1_ROS2_COMPAT_UV_VERSION})
  if(NOT EXISTS ${_uv_dir}/uv)
    execute_process(
      COMMAND sh -c "curl -LsSf https://astral.sh/uv/${JSK_ROS1_ROS2_COMPAT_UV_VERSION}/install.sh | env UV_UNMANAGED_INSTALL=${_uv_dir} UV_NO_MODIFY_PATH=1 sh"
      RESULT_VARIABLE _ret)
    if(NOT _ret EQUAL 0 OR NOT EXISTS ${_uv_dir}/uv)
      message(FATAL_ERROR "failed to install uv ${JSK_ROS1_ROS2_COMPAT_UV_VERSION}")
    endif()
  endif()
  install(PROGRAMS ${_uv_dir}/uv DESTINATION lib/${PROJECT_NAME})
  install(FILES cmake/catkin_uv.cmake DESTINATION share/${PROJECT_NAME}/cmake)
endif()

if("$ENV{ROS_VERSION}" EQUAL 1)
  include_directories(include ${catkin_INCLUDE_DIRS})

  install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )

  if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    add_rostest(test/test_rospy_rclpy_compat.test)
  endif()
  return()
endif()

install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
)
ament_export_include_directories(include)
ament_export_dependencies(rclcpp)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_rospy_rclpy_compat test/test_rospy_rclpy_compat.py
    TIMEOUT 120)
endif()

ament_package(CONFIG_EXTRAS cmake/${PROJECT_NAME}-extras.cmake)
