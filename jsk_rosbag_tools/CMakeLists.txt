cmake_minimum_required(VERSION 2.8.3)
project(jsk_rosbag_tools)

find_package(
  catkin REQUIRED
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
)

file(GLOB SCRIPTS_FILES scripts/*)
catkin_install_python(
  PROGRAMS ${SCRIPTS_FILES}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_download(download_audio_data
  https://drive.google.com/uc?export=download&id=1rFZYoFjLqIWjEe0DaNiL3k9893m31nu7
  DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/samples/data
  FILENAME 2022-05-07-hello-test.bag
  MD5 3650e27dad2c7dc0e447033259290db6
)
catkin_download(download_video_data
  https://drive.google.com/uc?export=download&id=1v4YNOHnHYxLOty1lYR2R6lfNF0itCwK7
  DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/samples/data
  FILENAME 20220530173950_go_to_kitchen_rosbag.bag
  MD5 d51fa8aeacd36f7aaa1597b67bd9ffdf
)
add_custom_target(download ALL DEPENDS download_audio_data download_video_data)

install(FILES requirements.in requirements.txt
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS roslint rostest)

  set(python_test_scripts
    tests/test_jsk_rosbag_tools.py
  )

  roslint_python()
  roslint_python(${python_test_scripts})
  roslint_add_test()

  catkin_install_python(PROGRAMS ${python_test_scripts}
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

  add_rostest(tests/test_jsk_rosbag_tools.test
    DEPENDENCIES download_audio_data download_video_data)
endif()
