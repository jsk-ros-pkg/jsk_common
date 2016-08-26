/*
 * test_nodelet_log.cpp
 * Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>
 */

#include <gtest/gtest.h>
#include <nodelet/nodelet.h>
#include <nodelet/loader.h>
#include <ros/ros.h>
#include <string>
#include <vector>

TEST(JSKNodeletLog, TEST_LOG)
{
  nodelet::Loader n(false);
  ros::M_string remappings;
  std::string nodelet_name = ros::this_node::getName();
  std::string nodelet_type = "jsk_topic_tools_test/LogUtils";
  std::vector<std::string> argv;
  EXPECT_TRUE(n.load(nodelet_name, nodelet_type, remappings, argv));
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_nodelet_log_utils");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
