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
  CFG_EXTRAS download_package.cmake
  )
catkin_add_env_hooks(99.jsk_tools SHELLS sh bash zsh DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)
catkin_add_env_hooks(99.jsk_tools-completion SHELLS bash zsh DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)

install(DIRECTORY src test bin dot-files
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  )
install(DIRECTORY bin/
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  USE_SOURCE_PERMISSIONS
  )

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
  roslint_add_test()
  find_package(rostest REQUIRED)
  # check ros distro >= indigo
  execute_process(
    COMMAND python -c "import sys; sys.exit(ord('$ENV{ROS_DISTRO}'[0]) >= ord('indigo'[0]))"
    RESULT_VARIABLE DISTRO_GE_INDIGO)
  if(${DISTRO_GE_INDIGO})
    # FIXME: jsk_tools/test_topic_published.py does not work on hydro travis/jenkins
    # https://github.com/jsk-ros-pkg/jsk_common/pull/1293#issuecomment-164158260
    add_rostest(test/test_topic_published.launch)
  endif()
  add_rostest(test/test_stdout.test)
endif()
