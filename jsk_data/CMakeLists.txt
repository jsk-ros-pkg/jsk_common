cmake_minimum_required(VERSION 2.8.3)
project(jsk_data)
find_package(catkin REQUIRED)
catkin_package(
)

catkin_python_setup()
catkin_install_python(PROGRAMS scripts/jsk_data DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_add_env_hooks(99.jsk_data-completion SHELLS bash DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)

install(DIRECTORY launch rvizconfig scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

if (CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test)
endif()
