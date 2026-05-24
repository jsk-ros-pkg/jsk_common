cmake_minimum_required(VERSION 2.8.3)
project(app_scheduler)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    message_generation
)

catkin_python_setup()

add_message_files(
    FILES
    AppSchedule.msg
    AppScheduleEntry.msg
    AppScheduleEntries.msg
)

add_service_files(
    FILES
    AddEntry.srv
    RemoveEntry.srv
)

generate_messages()

catkin_package(
    CATKIN_DEPENDS message_runtime
)

install(DIRECTORY apps launch sample scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)
