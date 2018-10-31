cmake_minimum_required(VERSION 2.8.3)
project(jsk_tools)

find_package(catkin REQUIRED)
catkin_python_setup()
set(ROS_BUILD_TYPE RelWithDebInfo)
catkin_package(
  CATKIN_DEPENDS #
  LIBRARIES #
  INCLUDE_DIRS #
  DEPENDS #
  CFG_EXTRAS download_package.cmake shell_test.cmake
  )
catkin_add_env_hooks(99.config_switch SHELLS bash zsh DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)
catkin_add_env_hooks(99.jsk_tools SHELLS sh bash zsh DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)
catkin_add_env_hooks(99.jsk_tools-completion SHELLS bash zsh DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)

set(BIN_EXECUTABLES git-jsk-commit ros_console.py run_tmux)
foreach(exec ${BIN_EXECUTABLES})
  add_custom_command(OUTPUT ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_BIN_DESTINATION}/${exec}
    COMMAND cmake -E make_directory ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_BIN_DESTINATION}
    COMMAND cmake -E copy ${PROJECT_SOURCE_DIR}/bin/${exec} ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_BIN_DESTINATION}/
    DEPENDS ${PROJECT_SOURCE_DIR}/bin/${exec})
  add_custom_target(install_${exec} ALL DEPENDS ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_BIN_DESTINATION}/${exec})
  install(PROGRAMS bin/${exec} DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
endforeach(exec)

if (CATKIN_ENABLE_TESTING)
  find_package(roslint REQUIRED)
  roslint_python(src/jsk_tools/cltool.py)
  roslint_python(src/test_topic_published.py)
  roslint_python(src/test_rosparam_set.py)
  roslint_python(src/test_stdout.py)
  roslint_python(src/rostopic_host_sanity)
  roslint_add_test()
  find_package(rostest REQUIRED)
  macro(jsk_tools_add_rostest file)
    list(APPEND jsk_tools_test_files ${file})
    add_rostest(${file})
  endmacro()
  if("$ENV{ROS_DISTRO}" STRGREATER "hydro")
    # FIXME: jsk_tools/test_topic_published.py does not work on hydro travis/jenkins
    # https://github.com/jsk-ros-pkg/jsk_common/pull/1293#issuecomment-164158260
    jsk_tools_add_rostest(test/test_topic_published.launch)
    # FIXME: jsk_tools/sanity_lib.py does not work on hydro travis/jenkins
    # https://github.com/jsk-ros-pkg/jsk_common/pull/1405#issuecomment-225218625
    jsk_tools_add_rostest(test/python/sanity_lib.test)
    # FIXME: jsk_tools/test_topic_published.py does not work on hydro travis/jenkins
    # https://github.com/jsk-ros-pkg/jsk_common/pull/1293#issuecomment-164158260
    jsk_tools_add_rostest(test/test_rosparam_set.test)
  endif()
  jsk_tools_add_rostest(test/test_stdout.test)
  find_package(jsk_tools REQUIRED)
  jsk_tools_add_shell_test(COMMAND echo "testing jsk_tools_add_shell_test")
endif()

# install test files that really works
# From https://github.com/jsk-ros-pkg/jsk_recognition/pull/2345
# Install header files directly into ${CATKIN_PACKAGE_INCLUDE_DESTINATION}.
# If the package has setup.py and modules under src/${PROJECT_NAME}/,
# install python executables directly into ${CATKIN_PACKAGE_BIN_DESTINATION}.
# However, if it doesn't have setup.py, directories including python executables
# should be installed recursively into ${CATKIN_PACKAGE_SHARE_DESTINATION}.
# Also, other directories like 'launch' or 'sample' must be installed
# recursively into ${CATKIN_PACKAGE_SHARE_DESTINATION}.
# Be careful that 'launch' and 'launch/' are different: the former is directory
# and the latter is each content.
install(FILES ${jsk_tools_test_files}
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/test)
install(DIRECTORY test
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  PATTERN "*.test" EXCLUDE
  )
install(DIRECTORY src dot-files cmake launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  )
install(DIRECTORY bin/
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  USE_SOURCE_PERMISSIONS
  )
