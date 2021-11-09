cmake_minimum_required(VERSION 3.0.2)
project(jsk_image_capture)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    message_generation
)

catkin_python_setup()

add_service_files(
  FILES
  Capture.srv
)

generate_messages(
  DEPENDENCIES
)

catkin_package(
)
