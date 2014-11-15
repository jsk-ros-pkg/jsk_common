# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(posedetection_msgs)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs geometry_msgs cv_bridge message_generation )
find_package(OpenCV REQUIRED)

include_directories(SYSTEM ${catkin_INCLUDE_DIRS}
                           ${OpenCV_INCLUDE_DIRS})

# to compatible with fuerte directories
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

add_executable(feature0d_view src/feature0d_view.cpp)
target_link_libraries(feature0d_view ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(feature0d_view posedetection_msgs_gencpp)


add_message_files(FILES
  Curve1D.msg
  Feature0D.msg
  Feature1D.msg
  ImageFeature0D.msg
  ImageFeature1D.msg
  Object6DPose.msg
  ObjectDetection.msg
)

add_service_files(FILES
  Detect.srv
  Feature0DDetect.srv
  Feature1DDetect.srv
  TargetObj.srv
)

# to compatible with fuerte directories
generate_messages(
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs
)

catkin_package(
    CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs cv_bridge message_runtime
    DEPENDS OpenCV
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

install(TARGETS feature0d_view
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

