macro(download_package_for_groovy package_name package_url package_branch)
  if(COMMAND rosbuild_find_ros_package)
    rosbuild_find_ros_package(${package_name})
    if(EXISTS "${${package_name}_PACKAGE_PATH}")
      set(${package_name}_FOUND TRUE)
    endif()
  else()
    find_package(${package_name} QUIET)
  endif()
  if(NOT ${package_name}_FOUND)
    string(RANDOM _random)
    set(_output_directory /tmp/${PROJECT_NAME}_${package_name}_${_random})
    if(NOT EXISTS "${PROJECT_SOURCE_DIR}/../${package_name}")
      # git clone
      execute_process(
	COMMAND git clone --depth 1 -b ${package_branch} ${package_url} ${_output_directory}
	OUTPUT_VARIABLE _download_output
	ERROR_VARIABLE  _download_error
	RESULT_VARIABLE _download_failed)
      message("download ${package_name} files ${_download_output}")
      if (_download_failed)
        file(REMOVE_RECURSE ${_output_directory})
	message(FATAL_ERROR "Download ${package_name} failed : ${_download_failed}  ${_download_error}")
      endif(_download_failed)

      # copy
      if (EXISTS ${_output_directory}/${package_name}) # in case of stack 
	execute_process(
	  COMMAND cmake -E copy_directory  ${_output_directory}/${package_name}/ ${PROJECT_SOURCE_DIR}/../${package_name}
	  OUTPUT_VARIABLE _copy_output ERROR_VARIABLE  _copy_error RESULT_VARIABLE _copy_failed)
      else() # in case of package
	execute_process(
	  COMMAND cmake -E copy_directory  ${_output_directory}/                 ${PROJECT_SOURCE_DIR}/../${package_name}
	  OUTPUT_VARIABLE _copy_output ERROR_VARIABLE  _copy_error RESULT_VARIABLE _copy_failed)
      endif()
      message("copy ${PROJECT_SOURCE_DIR}/../${package_name} files ${_copy_output}")
      if (_copy_failed)
        file(REMOVE_RECURSE ${_output_directory})
	message(FATAL_ERROR "Copy ${package_name} failed : ${_copy_failed}  ${_copy_error}")
      endif(_copy_failed)
      file(REMOVE_RECURSE ${_output_directory})
    endif()

    # rospack profile
    execute_process(COMMAND rospack profile OUTPUT_VARIABLE _profile_output RESULT_VARIABLE _profile_failed)
    message("rospack profile ${_profile_output}")
    if (_compile_failed)
      message(FATAL_ERROR "rospack profile failed : ${_profile_failed}")
    endif(_compile_failed)
    # rosmake ${package_name}
    set(ENV{ROS_PACKAGE_PATH} ${PROJECT_SOURCE_DIR}/../${package_name}:$ENV{ROS_PACKAGE_PATH})
    execute_process(COMMAND rosmake ${package_name} OUTPUT_VARIABLE _compile_output ERROR_VARIABLE _compile_error RESULT_VARIABLE _compile_failed)
    message("compile ${package_name} ${_compile_output}")
    if (_compile_failed)
      file(REMOVE_RECURSE ${_output_directory})
      message(FATAL_ERROR "compile ${package_name} failed : ${_compile_failed} ${_compile_output} ${_compile_error}")
    endif(_compile_failed)

    message("sed -i s@'<!--\\s*\\(.*${package_name}.*/\\)\\s*-->'@'<\\1>'@g ${PROJECT_SOURCE_DIR}/manifest.xml")
    execute_process(
      COMMAND sh -c "sed -i s@'<!--\\s*\\(.*${package_name}.*/\\)\\s*-->'@'<\\1>'@g ${PROJECT_SOURCE_DIR}/manifest.xml"
      )
    include_directories(${PROJECT_SOURCE_DIR}/../${package_name}/include)
    link_directories(${PROJECT_SOURCE_DIR}/../${package_name}/lib)
    message("download_package_for_groovy ${package_name} ${package_url} ${package_branch} done")
  else() # if package is already installed
    include_directories(${${package_name}_PACKAGE_PATH}/include)
    link_directories(${${package_name}_PACKAGE_PATH}/lib)
  endif()
endmacro()