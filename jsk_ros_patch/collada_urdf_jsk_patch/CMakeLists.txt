# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(collada_urdf_jsk_patch)

find_package(catkin REQUIRED COMPONENTS collada_urdf)

catkin_package(CATKIN_DEPENDS collada_urdf)

execute_process(COMMAND make -C ${PROJECT_SOURCE_DIR})
execute_process(COMMAND cmake -E make_directory ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION})
execute_process(COMMAND cmake -E copy ${PROJECT_SOURCE_DIR}/urdf_to_collada ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION})




