if("$ENV{ROS_Distributions}" STREQUAL "electric" OR "$ENV{ROS_DISTRO}" STREQUAL "fuerte" OR "$ENV{ROS_DISTRO}" STREQUAL "hydro")
  message("-- pr2_groovy_patches package is for groovy")
  return()
endif()

cmake_minimum_required(VERSION 2.8.3)
project(pr2_groovy_patches)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)

macro(patch package)
  set(PATCH_BATCH_FILE ${PROJECT_SOURCE_DIR}/${package}_patch.bat )
  file(WRITE ${PATCH_BATCH_FILE} "set -x\npatch -p0 -N -i ${PROJECT_SOURCE_DIR}/patches/${package}.patch")
  execute_process(COMMAND rospack find ${package} OUTPUT_VARIABLE ${package}_PACKAGE_PATH OUTPUT_STRIP_TRAILING_WHITESPACE)
  execute_process( COMMAND cmake -E chdir ${${package}_PACKAGE_PATH} sh ${PATCH_BATCH_FILE} RESULT_VARIABLE RESULT_PATCH )
  message( STATUS "Patch result: ${RESULT_PATCH}, SOURCE DIR: ${${package}_PACKAGE_PATH}" )
  file(REMOVE ${PATCH_BATCH_FILE})
endmacro(patch package)

patch(face_detector_mono) 

catkin_package(
)

