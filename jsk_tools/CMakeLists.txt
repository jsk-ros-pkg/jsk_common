if($ENV{ROS_VERSION} EQUAL 2)
  cmake_minimum_required(VERSION 3.8)
  project(jsk_tools)

  # NOTE: audible_warning.py (this package's flagship node) and
  # battery_capacity_summary.py are ROS2-ported, along with the
  # ros_compat-backed src/jsk_tools/ python package. The cross-package
  # rostest helpers (test_topic_published.py, test_rosparam_set.py,
  # test_stdout.py) and sanity_diagnostics.py/roscore_*.py rely on
  # rostest/ROS1-master-specific introspection (rosnode/rostopic) with no
  # straightforward ROS2 equivalent, and stay ROS1-only for now.
  find_package(ament_cmake REQUIRED)
  find_package(ament_cmake_python REQUIRED)

  ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR src/${PROJECT_NAME})

  # The checked-in scripts use `#!/usr/bin/env python`, which resolves on
  # every ROS1 distro this package still supports (python2 on
  # indigo/kinetic/melodic, python3 on noetic) but not on ROS2: Ubuntu
  # 22.04+/24.04 base images (used by jazzy+) ship no bare `python`
  # binary at all. Rewrite just the installed ROS2 copies' shebang to
  # python3 instead of changing the shared source file, so ROS1's
  # rostest (test/test_audible_warning.test, run on all those distros)
  # keeps using the original, environment-appropriate interpreter.
  foreach(_script node_scripts/audible_warning.py bin/battery_capacity_summary.py)
    get_filename_component(_script_name ${_script} NAME)
    set(_script_out ${CMAKE_CURRENT_BINARY_DIR}/${_script_name})
    file(READ ${_script} _script_content)
    string(REPLACE "#!/usr/bin/env python\n" "#!/usr/bin/env python3\n" _script_content "${_script_content}")
    file(WRITE ${_script_out} "${_script_content}")
    # colcon --symlink-install symlinks straight to this file rather than
    # going through install()'s own chmod, so it needs +x set directly.
    execute_process(COMMAND chmod +x ${_script_out})
  endforeach()

  install(PROGRAMS
    ${CMAKE_CURRENT_BINARY_DIR}/audible_warning.py
    ${CMAKE_CURRENT_BINARY_DIR}/battery_capacity_summary.py
    DESTINATION lib/${PROJECT_NAME}
  )

  if(BUILD_TESTING)
    find_package(launch_testing_ament_cmake REQUIRED)
    add_launch_test(test/audible_warning_launch_test.py)
  endif()

  ament_package()

else() # if($ENV{ROS_VERSION} EQUAL 2)
cmake_minimum_required(VERSION 2.8.3)
project(jsk_tools)

find_package(catkin REQUIRED
  COMPONENTS
  dynamic_reconfigure)
catkin_python_setup()
set(ROS_BUILD_TYPE RelWithDebInfo)

generate_dynamic_reconfigure_options(
  cfg/AudibleWarning.cfg
)

catkin_package(
  CATKIN_DEPENDS dynamic_reconfigure #
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
  if("${exec}" MATCHES "\\.py$")
    catkin_install_python(PROGRAMS bin/${exec} DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
  else()
    install(PROGRAMS bin/${exec} DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
  endif()
endforeach(exec)

if (CATKIN_ENABLE_TESTING)
  find_package(roslint REQUIRED)
  roslint_python(node_scripts/audible_warning.py)
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
  jsk_tools_add_rostest(test/test_audible_warning.test)
  jsk_tools_add_rostest(test/test_stdout.test)
  jsk_tools_add_rostest(test/test_rostopic_host_sanity.test)
  jsk_tools_add_rostest(test/test_sanity_diagnostics.test)
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
install(DIRECTORY src dot-files cmake launch sample node_scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  )
install(DIRECTORY bin/
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  USE_SOURCE_PERMISSIONS
  )

endif() # if($ENV{ROS_VERSION} EQUAL 2)
