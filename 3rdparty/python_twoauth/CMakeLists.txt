cmake_minimum_required(VERSION 2.8.3)
project(python_twoauth)

find_package(catkin REQUIRED COMPONENTS
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES test
#  CATKIN_DEPENDS
#  DEPENDS system_lib
)

catkin_install_python(PROGRAMS bin/all_tweet.py bin/delete_all_dm.py bin/followingfollowers.py bin/followingfollowers_ids.py bin/get_access_token.py bin/home_timeline.py bin/oauth_header.py bin/streamingapi.py bin/twitpic.py bin/update_with_media.py bin/userstreams.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})