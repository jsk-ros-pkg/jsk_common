# for catkin only
cmake_minimum_required(VERSION 2.8.3)
project(assimp_devel)
find_package(catkin REQUIRED)

configure_file(${PROJECT_SOURCE_DIR}/Makefile ${CMAKE_CURRENT_BINARY_DIR}/Makefile)
execute_process(
#COMMAND cmake -E chdir ${PROJECT_SOURCE_DIR} make -f Makefile INSTALL_DIR=${CATKIN_DEVEL_PREFIX} GIT_DIR=${CMAKE_CURRENT_BINARY_DIR}/assimp_git CURRENT_DIR=
COMMAND cmake -E chdir ${CMAKE_CURRENT_BINARY_DIR} make -f ${PROJECT_SOURCE_DIR}/Makefile INSTALL_DIR=${CATKIN_DEVEL_PREFIX} GIT_PATCH=${PROJECT_SOURCE_DIR}/assimp_git.unit.patch\ ${PROJECT_SOURCE_DIR}/assimp_git.technique.patch\ ${PROJECT_SOURCE_DIR}/assimp_git.obj_export.patch
RESULT_VARIABLE _make_failed
)

if (_make_failed)
  message(FATAL_ERROR "Compile assimp_devel failed: ${_make_failed}")
endif(_make_failed)

# fake catkin package
add_library(assimp  SHARED IMPORTED)
set_target_properties(assimp PROPERTIES IMPORTED_IMPLIB ${CATKIN_DEVEL_PREFIX}/lib/libassimp3.so )
# fake install directory for catkin_package
file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include)
if($ENV{ROS_DISTRO} STREQUAL "groovy")
  catkin_package(
    INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include
    LIBRARIES    assimp
    )
  set(${PROJECT_NAME}_EXPORTED_TARGETS libsiftfast)
else()
  catkin_package(
    INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include
    LIBRARIES    assimp
    EXPORTED_TARGETS libassimp
    )
endif()
#file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include)
#file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include/assimp3)
# catkin_package(
#     DEPENDS 
#     CATKIN-DEPENDS 
#     INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include
#     LIBRARIES assimp3
# )

#install(PROGRAMS ${CATKIN_DEVEL_PREFIX}/bin/siftfast
#        DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

file(GLOB _libs ${CATKIN_DEVEL_PREFIX}/lib/libassimp.*)
install(FILES ${_libs}
        DESTINATION ${CATKIN_GLOBAL_LIB_DESTINATION})
install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/include/assimp
        DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/share/assimp
        DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}
        USE_SOURCE_PERMISSIONS)
