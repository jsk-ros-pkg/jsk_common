# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(kinect_near_mode_calibration)

# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS roscpp roslib openni_camera image_geometry)
find_package(OpenCV REQUIRED)
find_package(Eigen REQUIRED)


# TODO: fill in what other packages will need to use this package
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    DEPENDS OpenCV Eigen
    CATKIN-DEPENDS roscpp roslib openni_camera image_geometry
    #INCLUDE_DIRS # TODO include
    LIBRARIES  ${PROJECT_NAME}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  )
add_executable(calibrate src/calibrate.cpp)
add_executable(plot_data src/plot_data.cpp)
add_executable(acquire_data_command src/acquire_data_command.cpp)
target_link_libraries(calibrate ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(plot_data ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(acquire_data_command ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

install(TARGETS calibrate plot_data acquire_data_command
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)



