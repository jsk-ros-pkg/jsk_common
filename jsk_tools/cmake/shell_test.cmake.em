@[if DEVELSPACE]@
# set path to shell-test script in develspace
set(shell_test_script "python$ENV{ROS_PYTHON_VERSION} @(CMAKE_CURRENT_SOURCE_DIR)/cmake/run_shell_test.py")
@[else]@
# set path to shell-test script installspace
set(shell_test_script "python$ENV{ROS_PYTHON_VERSION} ${jsk_tools_DIR}/run_shell_test.py")
@[end if]@

function(jsk_tools_add_shell_test)
  cmake_parse_arguments(_shell_test "" "" "COMMAND;DEPENDENCIES" ${ARGN})

  # command need to be a string
  string(REPLACE ";" " " _shell_test_COMMAND "${_shell_test_COMMAND}")
  # convert command to test name
  string(REPLACE " " "_" testname ${_shell_test_COMMAND})
  string(REPLACE "/" "_" testname ${testname})
  set(output_path ${CATKIN_TEST_RESULTS_DIR}/${PROJECT_NAME})
  set(output_file_name "shell-test_${testname}.xml")
  set(cmd "${CMAKE_COMMAND} -E make_directory ${output_path}")
  set(cmd ${cmd} "${shell_test_script} '${_shell_test_COMMAND}' ${testname} '${output_path}/${output_file_name}' ${_shell_test_UNPARSED_ARGUMENTS}")
  catkin_run_tests_target("shell-test" ${testname} ${output_file_name} COMMAND ${cmd} DEPENDENCIES ${_shell_test_DEPENDENCIES})
endfunction()
