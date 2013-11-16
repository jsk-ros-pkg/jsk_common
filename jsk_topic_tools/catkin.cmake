# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(jsk_topic_tools)

find_package(catkin REQUIRED COMPONENTS topic_tools)

add_message_files(
  FILES TopicInfo.msg
)

add_service_files(
  FILES List.srv Update.srv
)

include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})
add_executable(topic_buffer_server src/topic_buffer_server.cpp)
add_executable(topic_buffer_client src/topic_buffer_client.cpp)

generate_messages(
  DEPENDENCIES std_msgs
)

catkin_package(
    DEPENDS topic_tools
    CATKIN-DEPENDS std_msgs
    INCLUDE_DIRS
    LIBRARIES
)
