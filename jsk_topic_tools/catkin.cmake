# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(jsk_topic_tools)

find_package(catkin REQUIRED COMPONENTS topic_tools message_generation roscpp)

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

generate_messages()

catkin_package(
    DEPENDS
    CATKIN-DEPENDS topic_tools message_runtime
    INCLUDE_DIRS
    LIBRARIES
)
