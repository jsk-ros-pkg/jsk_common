if(NOT USE_ROSBUILD)
  cmake_minimum_required(VERSION 2.8.3)
  project(rostwitter)

  find_package(catkin REQUIRED COMPONENTS rospy std_msgs mk)

  catkin_python_setup()

  catkin_package()

  add_custom_target(python_twitter_download ALL
    DEPENDS ${PROJECT_SOURCE_DIR}/${CATKIN_PACKAGE_PYTHON_DESTINATION})
  add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/${CATKIN_PACKAGE_PYTHON_DESTINATION}
    DEPENDS Makefile
    COMMAND cmake -E chdir ${CMAKE_CURRENT_BINARY_DIR} make -f ${PROJECT_SOURCE_DIR}/Makefile.twitter MK_DIR=${mk_PREFIX}/share/mk INSTALL_DIR=${PROJECT_SOURCE_DIR})

  install(PROGRAMS bin/get_access_token.py scripts/tweet.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
  install(CODE
    "file(GLOB egg_directories FOLLOW_SYMLINKS \"${PROJECT_SOURCE_DIR}/${CATKIN_GLOBAL_PYTHON_DESTINATION}/*.egg/*\")
     foreach(_egg_dir \${egg_directories})
       if (IS_DIRECTORY \${_egg_dir} AND NOT \${_egg_dir} MATCHES \"EGG-INFO\")
         file(INSTALL \${_egg_dir}
                      DESTINATION ${CATKIN_GLOBAL_PYTHON_DESTINATION}
                      USE_SOURCE_PERMISSIONS)
       endif()
     endforeach()
     "
    )

else() ## rosbuild

cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()

add_custom_target(python_twitter_download ALL
  DEPENDS ${PROJECT_SOURCE_DIR}/lib/python2.7/dist-packages/twitter)
add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/lib/python2.7/dist-packages/twitter
  DEPENDS Makefile
  COMMAND make -f ${PROJECT_SOURCE_DIR}/Makefile.twitter)

endif()

