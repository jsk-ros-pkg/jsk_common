cmake_minimum_required(VERSION 2.8.3)
project(bayesian_belief_networks)

find_package(catkin REQUIRED COMPONENTS rospy std_msgs mk message_generation)

catkin_python_setup()

add_message_files(DIRECTORY msg FILES Result.msg Observation.msg)
add_service_files(DIRECTORY srv FILES Query.srv)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS std_msgs message_runtime)

if(EXISTS "/etc/debian_version")
  set(SETUPTOOLS_ARG_EXTRA "--install-layout=deb")
else()
  set(SETUPTOOLS_ARG_EXTRA "")
endif()

add_custom_target(python_baysian_belief_networks_download ALL
  DEPENDS ${PROJECT_SOURCE_DIR}/${CATKIN_GLOBAL_PYTHON_DESTINATION}/bayesian)
add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/${CATKIN_GLOBAL_PYTHON_DESTINATION}/bayesian
  DEPENDS Makefile.bayesian
  COMMAND cmake -E chdir ${CMAKE_CURRENT_BINARY_DIR} make -f ${PROJECT_SOURCE_DIR}/Makefile.bayesian MK_DIR=${mk_PREFIX}/share/mk INSTALL_DIR=${PROJECT_SOURCE_DIR} SETUPTOOLS_ARG_EXTRA=${SETUPTOOLS_ARG_EXTRA} DIST_DIR=${PROJECT_SOURCE_DIR}/${CATKIN_GLOBAL_PYTHON_DESTINATION})

install(DIRECTORY samples # examples
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)

install(DIRECTORY ${CATKIN_GLOBAL_PYTHON_DESTINATION}/bayesian/test ${CATKIN_GLOBAL_PYTHON_DESTINATION}/bayesian/examples
  DESTINATION ${CATKIN_GLOBAL_PYTHON_DESTINATION}/bayesian
  USE_SOURCE_PERMISSIONS)
