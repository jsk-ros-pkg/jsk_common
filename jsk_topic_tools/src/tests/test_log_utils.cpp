#include "jsk_topic_tools/log_utils.h"
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <gtest/gtest.h>


TEST(LogUtils, testWarnNoRemap){
  ros::NodeHandle pnh = ros::NodeHandle("~");
  ros::Publisher pub_remap = pnh.advertise<std_msgs::Header>(
    /*topic=*/"remap", /*queue_size=*/10);
  ros::Publisher pub_noremap = pnh.advertise<std_msgs::Header>(
    /*topic=*/"noremap", /*queue_size=*/10);

  std::vector<std::string> topics;
  topics.push_back(std::string("~remap"));
  bool actual;
  actual = jsk_topic_tools::warnNoRemap(topics);
  EXPECT_EQ(true, actual);

  topics.push_back(std::string("~noremap"));
  actual = jsk_topic_tools::warnNoRemap(topics);
  EXPECT_EQ(false, actual);
}

TEST(LogUtils, testGetFunctionName){
  std::string name = std::string(
    "virtual void jsk_topic_tools::ConnectionBasedNodelet::warnNeverSubscribedCallback(const ros::WallTimerEvent&)");
  std::string actual = jsk_topic_tools::getFunctionName(name);
  std::string expected = std::string("jsk_topic_tools::ConnectionBasedNodelet::warnNeverSubscribedCallback");
  ASSERT_STREQ(expected.c_str(), actual.c_str());
}

TEST(LogUtils, testJSKROSXXX){
  JSK_ROS_DEBUG("Testing JSK_ROS_DEBUG: %ld", ros::Time::now().toNSec());
  JSK_ROS_INFO("Testing JSK_ROS_INFO: %ld", ros::Time::now().toNSec());
  JSK_ROS_WARN("Testing JSK_ROS_WARN: %ld", ros::Time::now().toNSec());
  JSK_ROS_ERROR("Testing JSK_ROS_ERROR: %ld", ros::Time::now().toNSec());
  JSK_ROS_FATAL("Testing JSK_ROS_FATAL: %ld", ros::Time::now().toNSec());

  JSK_ROS_DEBUG_STREAM("Testing " << "JSK_ROS_DEBUG_STREAM: " << ros::Time::now().toNSec());
  JSK_ROS_INFO_STREAM("Testing " << "JSK_ROS_INFO_STREAM: " << ros::Time::now().toNSec());
  JSK_ROS_WARN_STREAM("Testing " << "JSK_ROS_WARN_STREAM: " << ros::Time::now().toNSec());
  JSK_ROS_ERROR_STREAM("Testing " << "JSK_ROS_ERROR_STREAM: " << ros::Time::now().toNSec());
  JSK_ROS_FATAL_STREAM("Testing " << "JSK_ROS_FATAL_STREAM: " << ros::Time::now().toNSec());

  JSK_ROS_DEBUG_THROTTLE(1, "Testing JSK_ROS_DEBUG_THROTTLE: %ld", ros::Time::now().toNSec());
  JSK_ROS_INFO_THROTTLE(1, "Testing JSK_ROS_INFO_THROTTLE: %ld", ros::Time::now().toNSec());
  JSK_ROS_WARN_THROTTLE(1, "Testing JSK_ROS_WARN_THROTTLE: %ld", ros::Time::now().toNSec());
  JSK_ROS_ERROR_THROTTLE(1, "Testing JSK_ROS_ERROR_THROTTLE: %ld", ros::Time::now().toNSec());
  JSK_ROS_FATAL_THROTTLE(1, "Testing JSK_ROS_FATAL_THROTTLE: %ld", ros::Time::now().toNSec());

  JSK_ROS_DEBUG_STREAM_THROTTLE(1, "Testing " << "JSK_ROS_DEBUG_STREAM_THROTTLE: " << ros::Time::now().toNSec());
  JSK_ROS_INFO_STREAM_THROTTLE(1, "Testing " << "JSK_ROS_INFO_STREAM_THROTTLE: " << ros::Time::now().toNSec());
  JSK_ROS_WARN_STREAM_THROTTLE(1, "Testing " << "JSK_ROS_WARN_STREAM_THROTTLE: " << ros::Time::now().toNSec());
  JSK_ROS_ERROR_STREAM_THROTTLE(1, "Testing " << "JSK_ROS_ERROR_STREAM_THROTTLE: " << ros::Time::now().toNSec());
  JSK_ROS_FATAL_STREAM_THROTTLE(1, "Testing " << "JSK_ROS_FATAL_STREAM_THROTTLE: " << ros::Time::now().toNSec());
}

int main(int argc, char **argv){
  ros::init(argc, argv, "test_log_utils");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
