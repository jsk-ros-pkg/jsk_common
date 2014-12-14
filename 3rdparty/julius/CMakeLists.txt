cmake_minimum_required(VERSION 2.8.3)
project(julius)

find_package(catkin REQUIRED)

catkin_package()

add_custom_command(
  OUTPUT julius_installed
  COMMAND make -f ${PROJECT_SOURCE_DIR}/Makefile.julius MD5SUM_DIR=${PROJECT_SOURCE_DIR} INSTALL_DIR=${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}
)

add_custom_command(
  OUTPUT grammar_kit_installed
  COMMAND make -f ${PROJECT_SOURCE_DIR}/Makefile.grammar-kit MD5SUM_DIR=${PROJECT_SOURCE_DIR} INSTALL_DIR=${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}
)

add_custom_command(
  OUTPUT dictation_kit_installed
  COMMAND make -f ${PROJECT_SOURCE_DIR}/Makefile.dictation-kit MD5SUM_DIR=${PROJECT_SOURCE_DIR} INSTALL_DIR=${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}
)

add_custom_target(all_installed ALL DEPENDS julius_installed grammar_kit_installed dictation_kit_installed)

install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/bin
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/bin
  USE_SOURCE_PERMISSIONS)
