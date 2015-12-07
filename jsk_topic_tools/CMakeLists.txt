# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(jsk_topic_tools)

# Use ccache if installed to make it fast to generate object files
find_program(CCACHE_FOUND ccache)
if(CCACHE_FOUND)
  set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
  set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
endif(CCACHE_FOUND)

find_package(catkin REQUIRED COMPONENTS topic_tools message_generation roscpp std_msgs std_srvs
  rostime
  nodelet tf
  geometry_msgs
  eigen_conversions
  topic_tools
  image_transport)
catkin_python_setup()
add_message_files(
  FILES TopicInfo.msg
)

add_service_files(
  FILES List.srv Update.srv ChangeTopic.srv PassthroughDuration.srv
)

generate_messages()
catkin_package(
    DEPENDS
    CATKIN_DEPENDS topic_tools message_runtime nodelet std_msgs std_srvs roscpp rostime
    INCLUDE_DIRS include
    LIBRARIES jsk_topic_tools
    CFG_EXTRAS nodelet.cmake
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -z defs")
#include_directories(${Boost_INCLUDE_DIRS})
add_executable(standalone_complexed_nodelet src/standalone_complexed_nodelet.cpp)
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
jsk_topic_tools_nodelet(src/lightweight_throttle_nodelet.cpp
  "jsk_topic_tools/LightweightThrottle" "lightweight_throttle")
jsk_topic_tools_nodelet(src/mux_nodelet.cpp
  "jsk_topic_tools/MUX" "mux")
jsk_topic_tools_nodelet(src/relay_nodelet.cpp
  "jsk_topic_tools/Relay" "relay")
jsk_topic_tools_nodelet(src/deprecated_relay_nodelet.cpp
  "jsk_topic_tools/DeprecatedRelay" "deprecated_relay")
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
add_dependencies(jsk_topic_tools ${PROJECT_NAME}_gencpp)
target_link_libraries(jsk_topic_tools ${catkin_LIBRARIES})

if (CATKIN_ENABLE_TESTING)
  find_package(roslint REQUIRED)
  roslint_cpp(src/log_utils.cpp)
  roslint_python(src/jsk_topic_tools)
  roslint_add_test()
  find_package(rostest REQUIRED)
  add_rostest(test/test_topic_buffer.test)
  add_rostest(test/test_topic_buffer_close_wait.test)
  add_rostest(test/test_topic_buffer_fixed_rate.test)
  add_rostest(test/test_topic_buffer_fixed_rate_and_update_rate.test)
  add_rostest(test/test_topic_buffer_update_rate.test)
  add_rostest(test/test_lightweight_throttle.test)
  add_rostest(test/test_topic_compare.test)
  add_rostest(test/test_hz_measure.test)
  add_rostest(test/test_block.test)
  add_rostest(test/test_connection_based_nodelet.test)
  add_rostest(test/test_connection_based_transport.test)
  catkin_add_nosetests(src/jsk_topic_tools/tests)
  catkin_add_gtest(test_rosparam_utils src/tests/test_rosparam_utils.cpp)
  target_link_libraries(test_rosparam_utils ${PROJECT_NAME})
  add_rostest_gtest(test_log_utils test/test_log_utils.launch src/tests/test_log_utils.cpp)
  target_link_libraries(test_log_utils ${PROJECT_NAME})
endif()

install(TARGETS topic_buffer_server topic_buffer_client jsk_topic_tools
  ${jsk_topic_tools_nodelet_executable_names}
  standalone_complexed_nodelet
  static_transform_pose_stamped
  transform_merger
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(FILES jsk_topic_tools_nodelet.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
# install(DIRECTORY include
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   )

install(DIRECTORY scripts launch test cmake config sample
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  )
