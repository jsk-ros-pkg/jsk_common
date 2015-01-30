cmake_minimum_required(VERSION 2.8.3)
project(rostwitter)

find_package(catkin REQUIRED COMPONENTS rospy std_msgs mk)

catkin_python_setup()

catkin_package()

install(PROGRAMS scripts/tweet.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
