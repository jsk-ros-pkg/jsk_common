# CMake helpers shared across ROS2-ported packages, available after
# find_package(jsk_ros1_ros2_compat REQUIRED).

# uv venv helpers are ROS2 only; ROS1 keeps using catkin_virtualenv
if("$ENV{ROS_VERSION}" EQUAL 2)
  include(${CMAKE_CURRENT_LIST_DIR}/catkin_uv.cmake)
endif()

# jsk_ros1_ros2_compat_install_python(
#   PROGRAMS <script>...
#   [DESTINATION <dir>]   # default: lib/${PROJECT_NAME}, for `ros2 run`
#   [USE_VENV])           # run with the venv of jsk_ros1_ros2_compat_generate_uv_venv()
#
# Installs python scripts whose source keeps the `#!/usr/bin/env python`
# shebang shared with ROS1 (python2 on indigo..melodic): the installed
# copy gets `#!/usr/bin/env python3` instead, since ROS2 base images
# ship no bare `python`, or the venv's python3 with USE_VENV.
function(jsk_ros1_ros2_compat_install_python)
  cmake_parse_arguments(ARG "USE_VENV" "DESTINATION" "PROGRAMS" ${ARGN})
  if(NOT ARG_DESTINATION)
    set(ARG_DESTINATION lib/${PROJECT_NAME})
  endif()
  set(_shebang "#!/usr/bin/env python3")
  if(ARG_USE_VENV)
    if(NOT ${PROJECT_NAME}_VENV_INSTALL_DIR)
      message(FATAL_ERROR "USE_VENV needs jsk_ros1_ros2_compat_generate_uv_venv() first")
    endif()
    set(_shebang "#!${${PROJECT_NAME}_VENV_INSTALL_DIR}/bin/python3")
  endif()
  foreach(_script ${ARG_PROGRAMS})
    get_filename_component(_script_abs ${_script} ABSOLUTE)
    get_filename_component(_script_name ${_script} NAME)
    # keep the generated path independent of DESTINATION, which may hold
    # generator expressions (e.g. lib/$<CONFIG>) that install() evaluates
    # in the source path too, or absolute/.. paths
    string(MD5 _destination_id "${ARG_DESTINATION}")
    set(_script_out ${CMAKE_CURRENT_BINARY_DIR}/jsk_ros1_ros2_compat_install_python/${_destination_id}/${_script_name})
    # re-run configure (and so regenerate this copy) when the source changes
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${_script_abs})
    file(READ ${_script_abs} _content)
    string(REGEX REPLACE "^#![^\n]*python[^\n]*\n" "${_shebang}\n" _content "${_content}")
    file(WRITE ${_script_out} "${_content}")
    # colcon --symlink-install symlinks straight to this file rather than
    # going through install()'s own chmod, so it needs +x set directly.
    execute_process(COMMAND chmod +x ${_script_out})
    install(PROGRAMS ${_script_out} DESTINATION ${ARG_DESTINATION})
  endforeach()
endfunction()
