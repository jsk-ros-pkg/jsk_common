# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(jsk_topic_tools)

# Use ccache if installed to make it fast to generate object files
if (CMAKE_VERSION VERSION_LESS 3.4)
  find_program(CCACHE_FOUND ccache)
  if(CCACHE_FOUND)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
  endif(CCACHE_FOUND)
endif()

find_package(catkin REQUIRED COMPONENTS
  diagnostic_updater
  dynamic_reconfigure
  eigen_conversions
  geometry_msgs
  image_transport
  message_generation
  nodelet
  roscpp
  rostime
  std_msgs
  std_srvs
  tf
  topic_tools
)

configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/include/jsk_topic_tools/nodelet_version.h.in
  ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}/jsk_topic_tools/nodelet_version.h)

# download and install sample data
catkin_download(download_sample_pr2_wrench-2022-05-02 https://drive.google.com/uc?id=1ym_fxRTNZOTp4PCXIq_spJwFktoOkLDK
  DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/sample/data
  FILENAME sample_pr2_wrench-2022-05-02.bag
  MD5 0e2d0d431f10875cb372c85faf32b91e
)
catkin_download(download_sample_tf https://drive.google.com/uc?id=15i7dPbMxrxR0j6KsH5mSgIWrNNTrYoV4
  DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/sample/data
  FILENAME sample_tf.bag
  MD5 13cd56d8d85cdf80139bbe5f7a9e2ef0
)
add_custom_target(download ALL DEPENDS download_extra_data)

catkin_python_setup()
add_message_files(
  FILES TopicInfo.msg
)
add_service_files(
  FILES List.srv Update.srv ChangeTopic.srv PassthroughDuration.srv
)
generate_dynamic_reconfigure_options(
  cfg/ConstantRateThrottle.cfg
  cfg/LightweightThrottle.cfg
  cfg/StealthRelay.cfg
  cfg/SynchronizedThrottle.cfg
)
generate_messages()
catkin_package(
    DEPENDS
    CATKIN_DEPENDS topic_tools message_runtime nodelet std_msgs std_srvs roscpp rostime
    INCLUDE_DIRS include
    LIBRARIES jsk_topic_tools
    CFG_EXTRAS nodelet.cmake
)

## Generate jsk_topic_tools_version.h.
# Note that catkin_package_xml needs to be called, but it is called in catkin_package.
# split version in parts and pass to extra file
string(REPLACE "." ";" jsk_topic_tools_VERSION_LIST "${jsk_topic_tools_VERSION}")
list(LENGTH jsk_topic_tools_VERSION_LIST _count)
if(NOT _count EQUAL 3)
  message(FATAL_ERROR "jsk_topic_tools version '${jsk_topic_tools_VERSION}' does not match 'MAJOR.MINOR.PATCH' pattern")
endif()
list(GET jsk_topic_tools_VERSION_LIST 0 jsk_topic_tools_VERSION_MAJOR)
list(GET jsk_topic_tools_VERSION_LIST 1 jsk_topic_tools_VERSION_MINOR)
list(GET jsk_topic_tools_VERSION_LIST 2 jsk_topic_tools_VERSION_PATCH)

configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/include/jsk_topic_tools/jsk_topic_tools_version.h.in
  ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}/jsk_topic_tools/jsk_topic_tools_version.h)


if(topic_tools_VERSION VERSION_GREATER "1.13.2")
  add_definitions("-Dtopic_tools_relay_stealth_EXISTS")
endif()

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
if(NOT APPLE)
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -z defs")
endif(NOT APPLE)
#include_directories(${Boost_INCLUDE_DIRS})
add_executable(standalone_complexed_nodelet src/standalone_complexed_nodelet.cpp src/log_utils.cpp)
add_executable(topic_buffer_server src/topic_buffer_server.cpp)
add_executable(topic_buffer_client src/topic_buffer_client.cpp)
add_executable(transform_merger src/transform_merger.cpp)
add_executable(static_transform_pose_stamped
  src/static_transform_pose_stamped.cpp)
target_link_libraries(transform_merger ${catkin_LIBRARIES})
target_link_libraries(static_transform_pose_stamped ${catkin_LIBRARIES})
add_dependencies(static_transform_pose_stamped ${PROJECT_NAME}_gencpp)
add_dependencies(topic_buffer_server ${PROJECT_NAME}_gencpp)
add_dependencies(topic_buffer_client ${PROJECT_NAME}_gencpp)
target_link_libraries(topic_buffer_server ${catkin_LIBRARIES})
target_link_libraries(topic_buffer_client ${catkin_LIBRARIES})
target_link_libraries(standalone_complexed_nodelet ${catkin_LIBRARIES})

include(${PROJECT_SOURCE_DIR}/cmake/nodelet.cmake)

macro(jsk_topic_tools_nodelet _nodelet_cpp _nodelet_class _single_nodelet_exec_name)
  jsk_nodelet(${_nodelet_cpp} ${_nodelet_class} ${_single_nodelet_exec_name}
    jsk_topic_tools_nodelet_sources jsk_topic_tools_nodelet_executable_names)
endmacro(jsk_topic_tools_nodelet _nodelet_cpp _nodelet_class _single_nodelet_exec_name)

# nodelet shared object
jsk_topic_tools_nodelet(src/constant_rate_throttle_nodelet.cpp
  "jsk_topic_tools/ConstantRateThrottle" "constant_rate_throttle")
jsk_topic_tools_nodelet(src/lightweight_throttle_nodelet.cpp
  "jsk_topic_tools/LightweightThrottle" "lightweight_throttle")
jsk_topic_tools_nodelet(src/mux_nodelet.cpp
  "jsk_topic_tools/MUX" "mux")
jsk_topic_tools_nodelet(src/relay_nodelet.cpp
  "jsk_topic_tools/Relay" "relay")
jsk_topic_tools_nodelet(src/deprecated_relay_nodelet.cpp
  "jsk_topic_tools/DeprecatedRelay" "deprecated_relay")
jsk_topic_tools_nodelet(src/stealth_relay_nodelet.cpp
  "jsk_topic_tools/StealthRelay" "stealth_relay")
jsk_topic_tools_nodelet(src/passthrough_nodelet.cpp
  "jsk_topic_tools/Passthrough" "passthrough")
jsk_topic_tools_nodelet(src/block_nodelet.cpp
  "jsk_topic_tools/Block" "block")
jsk_topic_tools_nodelet(src/snapshot_nodelet.cpp
  "jsk_topic_tools/Snapshot" "snapshot")
jsk_topic_tools_nodelet(src/hz_measure_nodelet.cpp
  "jsk_topic_tools/HzMeasure" "hz_measure")
jsk_topic_tools_nodelet(src/vital_checker_nodelet.cpp
  "jsk_topic_tools/VitalCheckerNodelet" "vital_checker")
jsk_topic_tools_nodelet(src/string_relay_nodelet.cpp
  "jsk_topic_tools/StringRelay" "string_relay")
jsk_topic_tools_nodelet(src/synchronized_throttle_nodelet.cpp
  "jsk_topic_tools/SynchronizedThrottle" "synchronized_throttle")

add_library(jsk_topic_tools SHARED
  ${jsk_topic_tools_nodelet_sources}
  src/timered_diagnostic_updater.cpp
  src/diagnostic_utils.cpp
  src/rosparam_utils.cpp
  src/time_accumulator.cpp
  src/vital_checker.cpp
  src/color_utils.cpp
  src/log_utils.cpp
  src/connection_based_nodelet.cpp
  src/diagnostic_nodelet.cpp
  src/series_boolean.cpp
  src/counter.cpp)
add_dependencies(jsk_topic_tools ${PROJECT_NAME}_gencpp ${PROJECT_NAME}_gencfg)
target_link_libraries(jsk_topic_tools ${catkin_LIBRARIES})

if (CATKIN_ENABLE_TESTING)
  find_package(roslint REQUIRED)
  set(ROSLINT_CPP_OPTS "--filter=-whitespace/parens,-whitespace/braces")
  roslint_cpp()
  roslint_python()
  macro(jsk_topic_tools_add_rostest file)
    list(APPEND jsk_topic_tools_test_files ${file})
    add_rostest(${file})
  endmacro()
  find_package(rostest REQUIRED)
  jsk_topic_tools_add_rostest(test/test_boolean_node.test)
  jsk_topic_tools_add_rostest(test/test_topic_buffer.test)
  jsk_topic_tools_add_rostest(test/test_topic_buffer_close_wait.test)
  # topic_buffer tests are tend to fail in GA because of CPU high load.
  # https://github.com/jsk-ros-pkg/jsk_common/pull/1739
  # jsk_topic_tools_add_rostest(test/test_topic_buffer_fixed_rate.launch)
  # jsk_topic_tools_add_rostest(test/test_topic_buffer_fixed_rate_and_update_rate.launch)
  # jsk_topic_tools_add_rostest(test/test_topic_buffer_update_rate.launch)
  jsk_topic_tools_add_rostest(test/test_constant_rate_throttle.test)
  jsk_topic_tools_add_rostest(test/test_lightweight_throttle.test)
  jsk_topic_tools_add_rostest(test/test_topic_compare.test)
  jsk_topic_tools_add_rostest(test/test_hz_measure.test)
  jsk_topic_tools_add_rostest(test/test_block.test)
  jsk_topic_tools_add_rostest(test/test_connection_based_nodelet.test)
  jsk_topic_tools_add_rostest(test/test_connection_based_transport.test)
  jsk_topic_tools_add_rostest(test/test_stealth_relay.test)
  jsk_topic_tools_add_rostest(test/test_synchronized_throttle.test)
  jsk_topic_tools_add_rostest(test/test_static_tf_republisher.test)
  jsk_topic_tools_add_rostest(test/test_pose_stamped_publisher.test)
  jsk_topic_tools_add_rostest(test/test_python_master_util.test)
  jsk_topic_tools_add_rostest(test/test_transform_wrench.test)
  catkin_add_nosetests(src/jsk_topic_tools/tests)
  catkin_add_gtest(test_rosparam_utils src/tests/test_rosparam_utils.cpp)
  target_link_libraries(test_rosparam_utils ${PROJECT_NAME})
  add_rostest_gtest(test_log_utils test/test_log_utils.launch src/tests/test_log_utils.cpp)
  target_link_libraries(test_log_utils ${PROJECT_NAME})

  # nodelets for testing
  add_library(jsk_topic_tools_test src/tests/test_log_utils_nodelet.cpp)
  target_link_libraries(jsk_topic_tools_test ${PROJECT_NAME})
  # testing with loading nodelet
  add_rostest_gtest(test_nodelet_log_utils test/test_nodelet_log_utils.test test/test_nodelet_log_utils.cpp)
  target_link_libraries(test_nodelet_log_utils ${PROJECT_NAME})
  add_dependencies(test_nodelet_log_utils jsk_topic_tools_test)
  if("$ENV{ROS_DISTRO}" STRGREATER "indigo")
    # rostest/publishtest is available on kinetic and later
    jsk_topic_tools_add_rostest(test/test_standalone_complexed_nodelet.test)
  elseif("$ENV{ROS_DISTRO}" STRGREATER "hydro")
    # FIXME: jsk_tools/test_topic_published.py does not work on hydro travis/jenkins
    # https://github.com/jsk-ros-pkg/jsk_common/pull/1293#issuecomment-164158260
    jsk_topic_tools_add_rostest(test/test_tf_to_pose.test)
  endif()
endif()

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
install(TARGETS topic_buffer_server topic_buffer_client jsk_topic_tools
  ${jsk_topic_tools_nodelet_executable_names}
  standalone_complexed_nodelet
  static_transform_pose_stamped
  transform_merger
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  jsk_topic_tools_nodelet.xml
  jsk_topic_tools_test_nodelet.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" # Ignore *_version.h.in files
)

file(GLOB PYTHON_SCRIPTS scripts/*.py)
list(APPEND PYTHON_SCRIPTS scripts/is_synchronized scripts/rostopic_connection_list)
catkin_install_python(
  PROGRAMS ${PYTHON_SCRIPTS}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  PROGRAMS scripts/sample_topic_update_caller.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES ${jsk_topic_tools_test_files}
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/test
)
install(DIRECTORY test
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  PATTERN "*.test" EXCLUDE
)
install(DIRECTORY cmake config launch sample
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
