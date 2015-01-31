# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(image_view2)

find_package(catkin REQUIRED COMPONENTS roscpp cv_bridge std_msgs sensor_msgs geometry_msgs image_transport tf image_geometry message_filters message_generation)

add_message_files(
  FILES ImageMarker2.msg PointArrayStamped.msg
)

add_service_files(
  FILES ChangeMode.srv
  )

include_directories(include ${catkin_INCLUDE_DIRS})

# Image viewers
find_package(OpenCV REQUIRED)
add_executable(image_view2 image_view2.cpp)
target_link_libraries(image_view2 ${OpenCV_LIBS} ${catkin_LIBRARIES})
add_dependencies(image_view2 ${PROJECT_NAME}_gencpp)

# Point Rectangle Extractor
add_executable(points_rectangle_extractor points_rectangle_extractor.cpp)
find_package(Boost REQUIRED COMPONENTS signals)
include_directories(${Boost_INCLUDE_DIRS})
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
target_link_libraries(points_rectangle_extractor ${Boost_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(points_rectangle_extractor ${PROJECT_NAME}_gencpp)

generate_messages(DEPENDENCIES geometry_msgs std_msgs)

catkin_package(
    DEPENDS OpenCV PCL
    CATKIN_DEPENDS roscpp cv_bridge std_msgs sensor_msgs geometry_msgs image_transport tf image_geometry message_filters
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

install(TARGETS image_view2 points_rectangle_extractor
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

