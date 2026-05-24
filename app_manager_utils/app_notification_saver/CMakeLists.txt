cmake_minimum_required(VERSION 2.8.3)
project(app_notification_saver)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
)

add_service_files(
  FILES SaveAppNotification.srv
)

catkin_python_setup()

generate_messages()

catkin_package(
  CATKIN_DEPENDS message_runtime
)
