# catkin_virtualenv-like python virtualenv generation with uv, for ROS2
# (catkin_virtualenv has no ROS2 release). Included by
# jsk_ros1_ros2_compat-extras.cmake.

# installed by jsk_ros1_ros2_compat's CMakeLists.txt, next to this file's share/
get_filename_component(JSK_ROS1_ROS2_COMPAT_UV_EXECUTABLE
  "${CMAKE_CURRENT_LIST_DIR}/../../../lib/jsk_ros1_ros2_compat/uv" ABSOLUTE)

# jsk_ros1_ros2_compat_generate_uv_venv(
#   [PYTHON_INTERPRETER <python>]   # default: python3
#   [NO_SYSTEM_SITE_PACKAGES]       # default: system site-packages (rclpy, ...) are visible
#   [CHECK_IMPORTS <module>...]     # python modules to import as a sanity check
#   [STRIP])                        # strip shared libraries in the venv
#
# The ROS2 counterpart of catkin_virtualenv's catkin_generate_virtualenv():
# creates a relocatable virtualenv from the pyproject.toml/uv.lock of the
# calling package with `uv sync --frozen`, at configure time, and installs
# it to share/${PROJECT_NAME}/venv, so that it is also shipped in the deb.
# uv itself is the pinned version that jsk_ros1_ros2_compat downloads and
# installs to its own lib/ at its build time (there is no rosdep key for
# uv), so packages using this get no extra build dependency on it.
#
# Sets ${PROJECT_NAME}_VENV_DIR (build tree) and
# ${PROJECT_NAME}_VENV_INSTALL_DIR (install tree).
function(jsk_ros1_ros2_compat_generate_uv_venv)
  cmake_parse_arguments(ARG "NO_SYSTEM_SITE_PACKAGES;STRIP" "PYTHON_INTERPRETER" "CHECK_IMPORTS" ${ARGN})
  if(NOT ARG_PYTHON_INTERPRETER)
    set(ARG_PYTHON_INTERPRETER python3)
  endif()
  # prefer the system python (/usr/bin) over e.g. ~/.local/bin ones, since
  # the venv must see the system site-packages of ROS; NO_CACHE so that
  # each call resolves its own PYTHON_INTERPRETER
  find_program(_python NAMES ${ARG_PYTHON_INTERPRETER} PATHS /usr/bin NO_DEFAULT_PATH NO_CACHE)
  if(NOT _python)
    find_program(_python NAMES ${ARG_PYTHON_INTERPRETER} NO_CACHE REQUIRED)
  endif()

  if(NOT EXISTS "${JSK_ROS1_ROS2_COMPAT_UV_EXECUTABLE}")
    message(FATAL_ERROR "uv is not found at ${JSK_ROS1_ROS2_COMPAT_UV_EXECUTABLE}, "
      "jsk_ros1_ros2_compat is built without it")
  endif()

  set(_venv ${CMAKE_CURRENT_BINARY_DIR}/venv)
  set(_uv env UV_CACHE_DIR=${CMAKE_CURRENT_BINARY_DIR}/uv_cache UV_PYTHON_DOWNLOADS=never UV_PROJECT_ENVIRONMENT=${_venv} ${JSK_ROS1_ROS2_COMPAT_UV_EXECUTABLE})
  set(_site --system-site-packages)
  if(ARG_NO_SYSTEM_SITE_PACKAGES)
    set(_site)
  endif()
  # re-run configure (and so `uv sync`) when the lock file changes
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/pyproject.toml ${CMAKE_CURRENT_SOURCE_DIR}/uv.lock)
  foreach(_cmd
      "venv;--relocatable;${_site};--python;${_python};--allow-existing;${_venv}"
      "sync;--frozen;--no-dev;--no-install-project")
    execute_process(COMMAND ${_uv} ${_cmd}
      WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} RESULT_VARIABLE _ret)
    if(NOT _ret EQUAL 0)
      message(FATAL_ERROR "uv ${_cmd} failed for ${PROJECT_NAME}")
    endif()
  endforeach()
  foreach(_module ${ARG_CHECK_IMPORTS})
    execute_process(COMMAND ${_venv}/bin/python3 -c "import ${_module}" RESULT_VARIABLE _ret)
    if(NOT _ret EQUAL 0)
      message(FATAL_ERROR "failed to import ${_module} in the uv venv of ${PROJECT_NAME}")
    endif()
  endforeach()
  if(ARG_STRIP AND CMAKE_STRIP)
    execute_process(COMMAND find ${_venv} -name "*.so*" -type f -exec ${CMAKE_STRIP} --strip-unneeded {} +)
  endif()

  install(DIRECTORY ${_venv}/
    DESTINATION share/${PROJECT_NAME}/venv
    USE_SOURCE_PERMISSIONS
    PATTERN ".lock" EXCLUDE)
  set(${PROJECT_NAME}_VENV_DIR ${_venv} PARENT_SCOPE)
  set(${PROJECT_NAME}_VENV_INSTALL_DIR ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/venv PARENT_SCOPE)
endfunction()
