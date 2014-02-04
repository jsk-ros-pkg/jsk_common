# for catkin only
cmake_minimum_required(VERSION 2.8.3)
project(assimp_devel)
find_package(catkin REQUIRED)
execute_process(
COMMAND cmake -E chdir ${PROJECT_SOURCE_DIR} make -f Makefile INSTALL_DIR=${CATKIN_DEVEL_PREFIX}
RESULT_VARIABLE _make_failed
)

if (_make_failed)
  message(FATAL_ERROR "Compile assimp_devel failed: ${_make_failed}")
endif(_make_failed)
