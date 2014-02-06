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
add_library(assimp3  SHARED IMPORTED)
set_target_properties(assimp3  PROPERTIES IMPORTED_IMPLIB ${CATKIN_DEVEL_PREFIX}/lib/libassimp3.so )

file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include)
file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include/assimp3)

catkin_package(
    DEPENDS 
    CATKIN-DEPENDS 
    INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include
    LIBRARIES assimp3
)
