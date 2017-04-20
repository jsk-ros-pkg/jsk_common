@[if DEVELSPACE]@
# set path to euslint script in develspace
set(euslint_script @(CMAKE_CURRENT_SOURCE_DIR)/bin/euslint)
set(shell_test_script @(CMAKE_CURRENT_SOURCE_DIR)/cmake/run_shell_test.py)
@[else]@
# set path to euslint script installspace
set(euslint_script ${jsk_tools_DIR}/euslint)
set(shell_test_script ${jsk_tools_DIR}/run_shell_test.py)
@[end if]@

function(euslint_add_test)
  cmake_parse_arguments(_euslint "" "TARGET" "OPTION" ${ARGN})
  if (NOT _euslint_TARGET)
    set(_euslint_TARGET "${ARGN}")
  endif()
  if (_euslint_TARGET STREQUAL "")
    set(_euslint_TARGET ".")
  endif()
  if (NOT _euslint_OPTION)
    set(_euslint_OPTION "")
  endif()
  get_filename_component(filepath "${_euslint_TARGET}" ABSOLUTE)
  set(euslint_cmd "${euslint_script} ${filepath} ${_euslint_OPTION}")
  string(REPLACE ";" " " testname ${_euslint_TARGET})
  string(REPLACE ";" " " testname ${filepath})
  string(REPLACE " " "_" testname ${testname})
  string(REPLACE "/" "_" testname ${testname})
  set(output_path ${CATKIN_TEST_RESULTS_DIR}/${PROJECT_NAME})
  set(output_file_name "euslint_${testname}.xml")
  set(cmd "${CMAKE_COMMAND} -E make_directory ${output_path}")
  set(cmd ${cmd} "${shell_test_script} '${euslint_cmd}' ${testname} '${output_path}/${output_file_name}'")
  catkin_run_tests_target("euslint" ${testname} ${output_file_name} COMMAND ${cmd})
endfunction()
