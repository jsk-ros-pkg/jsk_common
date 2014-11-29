cmake_minimum_required(VERSION 2.8.3)
project(downward)

find_package(catkin REQUIRED)

catkin_package()

add_custom_command(
  OUTPUT installed
  COMMAND make -f ${PROJECT_SOURCE_DIR}/Makefile
)
add_custom_command(
  OUTPUT ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
  COMMAND cmake -E copy_directory build/downward/scripts/ ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
  DEPENDS installed
)

add_custom_target(downward_install ALL DEPENDS ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts)

install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
  USE_SOURCE_PERMISSIONS)
