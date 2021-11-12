cmake_minimum_required(VERSION 2.8.3)

project(audio_video_recorder)

find_package(catkin REQUIRED COMPONENTS roscpp audio_common_msgs sensor_msgs)

find_package(PkgConfig)
pkg_check_modules(GST1.0 gstreamer-1.0 REQUIRED)

find_package(Boost REQUIRED COMPONENTS thread)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
  roscpp
  audio_common_msgs
  message_filters
  sensor_msgs
)

include_directories(include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${GST1.0_INCLUDE_DIRS}
)

add_executable(audio_video_recorder src/audio_video_recorder.cpp)
target_link_libraries(audio_video_recorder ${catkin_LIBRARIES} ${GST1.0_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(audio_video_recorder ${catkin_EXPORTED_TARGETS})

install(TARGETS audio_video_recorder
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch sample
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
