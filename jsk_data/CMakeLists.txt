cmake_minimum_required(VERSION 2.8.3)
project(jsk_data)
find_package(catkin REQUIRED COMPONENTS dynamic_reconfigure message_generation)

catkin_python_setup()

generate_dynamic_reconfigure_options(
  cfg/DataCollectionServer.cfg
)

catkin_add_env_hooks(99.jsk_data-completion SHELLS bash DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

# From https://github.com/jsk-ros-pkg/jsk_recognition/pull/2345
# Install header files directly into ${CATKIN_PACKAGE_INCLUDE_DESTINATION}.
# If the package has setup.py and modules under src/${PROJECT_NAME}/,
# install python executables directly into ${CATKIN_PACKAGE_BIN_DESTINATION}.
# However, if it doesn't have setup.py, directories including python executables
# should be installed recursively into ${CATKIN_PACKAGE_SHARE_DESTINATION}.
# Also, other directories like 'launch' or 'sample' must be installed
# recursively into ${CATKIN_PACKAGE_SHARE_DESTINATION}.
# Be careful that 'launch' and 'launch/' are different: the former is directory
# and the latter is each content.
catkin_install_python(PROGRAMS
  scripts/jsk_data
  scripts/robot_description_loader.py
  scripts/robot_description_saver.py
  scripts/rosbag_always.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY node_scripts/
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
install(PROGRAMS
  scripts/copy-pr2-bagfiles
  scripts/decompress
  scripts/gen-gif-all
  scripts/gopro-copy-files.sh
  scripts/hrp2_rosbag_always.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
if(${CMAKE_SYSTEM_PROCESSOR} MATCHES amd64* OR
   ${CMAKE_SYSTEM_PROCESSOR} MATCHES x86_64*)
  install(PROGRAMS
    scripts/drive-linux-x64
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
endif()
install(DIRECTORY launch rvizconfig sample tests
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  PATTERN tests/test_data_collection_server.py EXCLUDE
)
catkin_install_python(PROGRAMS
  tests/test_data_collection_server.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/tests/)

if (CATKIN_ENABLE_TESTING)
  find_package(roslint REQUIRED)
  roslint_python(src/jsk_data)
  roslint_add_test()

  catkin_add_nosetests(src/jsk_data/tests)

  find_package(roslaunch REQUIRED)
  find_package(baxter_description QUIET)
  if(baxter_description_FOUND)
    roslaunch_add_file_check(launch/baxter_play.launch)
    roslaunch_add_file_check(launch/baxter_record.launch)
  endif()
  find_package(fetch_description QUIET)
  if(fetch_description_FOUND)
    roslaunch_add_file_check(launch/fetch_play.launch)
    roslaunch_add_file_check(launch/fetch_record.launch)
  endif()
  find_package(pr2_description QUIET)
  if(pr2_description_FOUND)
    roslaunch_add_file_check(launch/pr2_play.launch)
  endif()
  if(0)
    # pr2_record could not run on travis
    # 1. environment variable 'ROS_ENV_LOADER' is not set.
    # 2. Invalid <param> tag: Cannot load command parameter [mk_save_dir]: command [mkdir -p /removable/bagfile/] returned with code [1].
    roslaunch_add_file_check(launch/pr2_record.launch)
  endif()
  roslaunch_add_file_check(launch/hrp2_record.launch)
  roslaunch_add_file_check(launch/urata_record.launch)

  find_package(rostest REQUIRED)
  add_rostest(tests/data_collection_server.test)
endif()
