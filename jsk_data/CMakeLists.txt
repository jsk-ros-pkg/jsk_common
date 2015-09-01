cmake_minimum_required(VERSION 2.8.3)
project(jsk_data)
find_package(catkin REQUIRED)
catkin_package(
)

catkin_add_env_hooks(99.jsk_data SHELLS sh bash zsh DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)
catkin_add_env_hooks(99.jsk_data-completion SHELLS bash zsh DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)

install(DIRECTORY launch rvizconfig scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

install(FILES Makefile
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
