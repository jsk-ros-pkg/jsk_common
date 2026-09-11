# Fall back to ROS1 when $ENV{ROS_VERSION} is unset (pre-ros_environment distros).
if("$ENV{ROS_VERSION}" STREQUAL "")
  set(ENV{ROS_VERSION} 1)
endif()

if("$ENV{ROS_VERSION}" EQUAL 1)
cmake_minimum_required(VERSION 2.8.3)
elseif("$ENV{ROS_VERSION}" EQUAL 2)
  cmake_minimum_required(VERSION 3.8)
endif()
project(jsk_rosbag_tools)

if("$ENV{ROS_VERSION}" EQUAL 1)
find_package(
  catkin REQUIRED
  catkin_virtualenv
)

if (${catkin_virtualenv_VERSION} VERSION_LESS "0.5.1")
  message(STATUS "jsk_rosbag_tools requires catkin_virtualenv >= 0.5.1")
  return()
endif()


catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
)
elseif("$ENV{ROS_VERSION}" EQUAL 2)
  find_package(ament_cmake REQUIRED)
  find_package(ament_cmake_python REQUIRED)
endif()

# ROS1 only: catkin_virtualenv builds an isolated venv from
# requirements.in*. Under ROS2 the same pip packages (moviepy, scipy,
# numpy, Pillow, tqdm, ...) are just expected to already be installed
# in the ambient python environment, as with jsk_tools/jsk_data.
if("$ENV{ROS_VERSION}" EQUAL 1)
if("$ENV{ROS_DISTRO}" STREQUAL "indigo" OR "$ENV{ROS_DISTRO}" STREQUAL "kinetic" OR "$ENV{ROS_DISTRO}" STREQUAL "melodic")
  file(MAKE_DIRECTORY $ENV{HOME}/.imageio/ffmpeg)
  file(DOWNLOAD https://github.com/imageio/imageio-binaries/raw/e7a7a2aad27decc79a3c2cd675ab4d0201e72e15/ffmpeg/ffmpeg-linux64-v3.3.1
    ${CMAKE_BINARY_DIR}/ffmpeg-linux64-v3.3.1)
  file (COPY ${CMAKE_BINARY_DIR}/ffmpeg-linux64-v3.3.1
    DESTINATION $ENV{HOME}/.imageio/ffmpeg/
    FILE_PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ)
  catkin_generate_virtualenv(
    INPUT_REQUIREMENTS requirements.in.python2
    PYTHON_INTERPRETER python2
  )
else()
find_package(Python COMPONENTS Interpreter)
if(Python_VERSION VERSION_GREATER_EQUAL 3.12.0)
  catkin_generate_virtualenv(
    INPUT_REQUIREMENTS requirements.in.python3.12
    PYTHON_INTERPRETER python3
  )
else()
  catkin_generate_virtualenv(
    INPUT_REQUIREMENTS requirements.in
    PYTHON_INTERPRETER python3
  )
endif()
endif()
endif()

file(GLOB SCRIPTS_FILES scripts/*)
if("$ENV{ROS_VERSION}" EQUAL 1)
set(_share_destination ${CATKIN_PACKAGE_SHARE_DESTINATION})
catkin_install_python(
  PROGRAMS ${SCRIPTS_FILES}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_download(download_audio_data
  https://drive.google.com/uc?export=download&id=1rFZYoFjLqIWjEe0DaNiL3k9893m31nu7
  DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/samples/data
  FILENAME 2022-05-07-hello-test.bag
  MD5 3650e27dad2c7dc0e447033259290db6
)
catkin_download(download_video_data
  https://drive.google.com/uc?export=download&id=1v4YNOHnHYxLOty1lYR2R6lfNF0itCwK7
  DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/samples/data
  FILENAME 20220530173950_go_to_kitchen_rosbag.bag
  MD5 d51fa8aeacd36f7aaa1597b67bd9ffdf
)
add_custom_target(download ALL DEPENDS download_audio_data download_video_data)

elseif("$ENV{ROS_VERSION}" EQUAL 2)
  set(_share_destination share/${PROJECT_NAME})
  ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR python/${PROJECT_NAME})
  install(PROGRAMS ${SCRIPTS_FILES}
    DESTINATION lib/${PROJECT_NAME}
  )
  # No sample-bag download target for ROS2: the launch test below
  # records its own small rosbag2 bag instead of relying on the ROS1
  # sample .bag files (a different, incompatible container format).
endif()

if("$ENV{ROS_VERSION}" EQUAL 1)
if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS roslint rostest)

  set(python_test_scripts
    tests/test_jsk_rosbag_tools.py
    tests/test_bag_to_video.py
  )

  roslint_python()
  roslint_python(${python_test_scripts})
  roslint_add_test()

  catkin_install_python(PROGRAMS ${python_test_scripts}
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

  add_rostest(tests/test_jsk_rosbag_tools.test
    DEPENDENCIES ${PROJECT_NAME}_generate_virtualenv download_audio_data download_video_data)
  if("$ENV{ROS_DISTRO}" STRGREATER "indigo")
    # could not install moviepy on indigo.
    add_rostest(tests/test_bag_to_video.test
      DEPENDENCIES ${PROJECT_NAME}_generate_virtualenv download_audio_data download_video_data)
  endif()
endif()
elseif("$ENV{ROS_VERSION}" EQUAL 2)
  if(BUILD_TESTING)
    # jsk_rosbag_tools has no long-running nodes to launch -- its
    # functionality is bag-file I/O, so a plain pytest test (rather
    # than launch_testing) is the right fit.
    find_package(ament_cmake_pytest REQUIRED)
    ament_add_pytest_test(test_jsk_rosbag_tools_ros2 tests/test_jsk_rosbag_tools_ros2.py)
  endif()
endif()

install(DIRECTORY scripts samples
  DESTINATION ${_share_destination}
  USE_SOURCE_PERMISSIONS
)

if("$ENV{ROS_VERSION}" EQUAL 1)
install(FILES requirements.txt
  DESTINATION ${_share_destination}
)
return()
endif()

ament_package()
