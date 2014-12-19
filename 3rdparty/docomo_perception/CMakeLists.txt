cmake_minimum_required(VERSION 2.8.3)
project(docomo_perception)

find_package(catkin REQUIRED message_generation std_msgs speech_recognition_msgs sensor_msgs)

add_message_files(
   FILES
   TextToSpeech.msg
   ImageRecognitionCandidate.msg
 )

add_service_files(
  FILES
  Chat.srv
  Knowledge.srv
  ImageRecognition.srv
  SentenseUnderstanding.srv
)

generate_messages(
   DEPENDENCIES
   std_msgs
   sensor_msgs
 )

catkin_package(
  CATKIN_DEPENDS message_runtime
)

#############
## Install ##
#############

install(
  DIRECTORY scripts/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
  USE_SOURCE_PERMISSIONS
)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)