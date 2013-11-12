# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(collada_urdf_jsk_patch)

find_package(catkin REQUIRED COMPONENTS collada_urdf)

catkin_package(CATKIN_DEPENDS collada_urdf)

execute_process(COMMAND make -C ${PROJECT_SOURCE_DIR})




