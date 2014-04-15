# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(jsk_topic_tools)

find_package(catkin REQUIRED COMPONENTS topic_tools message_generation roscpp rostest std_msgs
  nodelet
  topic_tools)

add_message_files(
  FILES TopicInfo.msg
)

add_service_files(
  FILES List.srv Update.srv
)

#include_directories(${Boost_INCLUDE_DIRS})
add_executable(topic_buffer_server src/topic_buffer_server.cpp)
add_executable(topic_buffer_client src/topic_buffer_client.cpp)
add_dependencies(topic_buffer_server ${PROJECT_NAME}_gencpp)
add_dependencies(topic_buffer_client ${PROJECT_NAME}_gencpp)
target_link_libraries(topic_buffer_server ${catkin_LIBRARIES})
target_link_libraries(topic_buffer_client ${catkin_LIBRARIES})

# nodelet shared object
include_directories(${catkin_INCLUDE_DIRS} include)
add_library(jsk_topic_tools SHARED src/lightweight_throttle_nodelet.cpp src/mux_nodelet.cpp)
target_link_libraries(jsk_topic_tools ${catkin_LIBRARIES})

generate_messages()

catkin_package(
    DEPENDS
    CATKIN_DEPENDS topic_tools message_runtime nodelet std_msgs
    INCLUDE_DIRS
    LIBRARIES
)

add_rostest(test/test_topic_buffer.test)
add_rostest(test/test_topic_buffer_close_wait.test)
add_rostest(test/test_topic_buffer_fixed_rate.test)
add_rostest(test/test_topic_buffer_fixed_rate_and_update_rate.test)
add_rostest(test/test_topic_buffer_update_rate.test)
add_rostest(test/test_lightweight_throttle.test)
add_rostest(test/test_topic_compare.test)

install(TARGETS topic_buffer_server topic_buffer_client jsk_topic_tools
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(FILES jsk_topic_tools_nodelet.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )

install(DIRECTORY scripts launch test DESTINATION
  ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  )
