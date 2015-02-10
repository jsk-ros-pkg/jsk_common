cmake_minimum_required(VERSION 2.8.3)
project(jsk_data)
find_package(catkin REQUIRED)
catkin_package(
)
install(DIRECTORY launch rvizconfig scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

install(FILES Makefile
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
