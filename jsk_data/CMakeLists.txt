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

catkin_install_python(PROGRAMS scripts/jsk_data DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch rvizconfig scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

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
  roslaunch_add_file_check(launch/pr2_play.launch)
  if(0)
    # pr2_record could not run on travis
    # 1. environment variable 'ROS_ENV_LOADER' is not set.
    # 2. Invalid <param> tag: Cannot load command parameter [mk_save_dir]: command [mkdir -p /removable/bagfile/] returned with code [1].
    roslaunch_add_file_check(launch/pr2_record.launch)
  endif()
  roslaunch_add_file_check(launch/hrp2_record.launch)
  roslaunch_add_file_check(launch/urata_record.launch)
endif()
