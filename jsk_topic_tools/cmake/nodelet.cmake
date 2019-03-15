# utility cmake for nodelet
# it will auto-generate an executable which contains a single nodelet.
macro(jsk_nodelet _nodelet_cpp _nodelet_class
    _single_nodelet_exec_name _CODE_PARAM_NAME)
  list(APPEND ${_CODE_PARAM_NAME} ${_nodelet_cpp})
  set(NODELET ${_nodelet_class})
  set(DEFAULT_NODE_NAME JSK_NODELET_${PROJECT_NAME}_${_single_nodelet_exec_name})
  if(${USE_ROSBUILD})
    rosbuild_find_ros_package(jsk_topic_tools)
    set(_jsk_topic_tools_SOURCE_DIR ${jsk_topic_tools_PACKAGE_PATH})
  else(${USE_ROSBUILD})
    if(jsk_topic_tools_SOURCE_DIR)
      set(_jsk_topic_tools_SOURCE_DIR ${jsk_topic_tools_SOURCE_DIR})
    elseif(jsk_topic_tools_SOURCE_PREFIX)
      set(_jsk_topic_tools_SOURCE_DIR ${jsk_topic_tools_SOURCE_PREFIX})
    else(jsk_topic_tools_SOURCE_PREFIX)
      set(_jsk_topic_tools_SOURCE_DIR ${jsk_topic_tools_PREFIX}/share/jsk_topic_tools)
    endif()
  endif(${USE_ROSBUILD})
  configure_file(
    ${_jsk_topic_tools_SOURCE_DIR}/cmake/single_nodelet_exec.cpp.in
    ${DEFAULT_NODE_NAME}.cpp)
  if(${USE_ROSBUILD})
    rosbuild_add_executable(${DEFAULT_NODE_NAME} build/${DEFAULT_NODE_NAME}.cpp)
  else(${USE_ROSBUILD})
    add_executable(${DEFAULT_NODE_NAME} ${DEFAULT_NODE_NAME}.cpp)
    target_link_libraries(${DEFAULT_NODE_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
  endif(${USE_ROSBUILD})
  set_target_properties(${DEFAULT_NODE_NAME} PROPERTIES OUTPUT_NAME ${_single_nodelet_exec_name})
  if (${ARGC} GREATER 4)
    list(APPEND ${ARGV4} ${DEFAULT_NODE_NAME})
  endif(${ARGC} GREATER 4)
  if(NOT ${USE_ROSBUILD})
    install(TARGETS ${DEFAULT_NODE_NAME}
      RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
      ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
  endif(NOT ${USE_ROSBUILD})
endmacro(jsk_nodelet _nodelet_cpp _nodelet_class
  _single_nodelet_exec_name CODE_PARAM_NAME)

find_package(Boost QUIET COMPONENTS program_options)
if(NOT Boost_FOUND)
  message(WARNING "Boost.program_options is not found. Using jsk_nodelet macro may fail.")
endif()
