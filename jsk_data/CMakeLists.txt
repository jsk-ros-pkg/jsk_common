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
endif()
