#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <jsk_topic_tools/log_utils.h>

namespace jsk_topic_tools
{
  class LogUtils : public nodelet::Nodelet
  {
  private:
    virtual void onInit()
      {
        JSK_NODELET_DEBUG("DEBUG output");
        JSK_NODELET_DEBUG_STREAM("DEBUG" << " output");
        JSK_NODELET_DEBUG_THROTTLE(10.0, "DEBUG output");
        JSK_NODELET_DEBUG_STREAM_THROTTLE(10.0, "DEBUG" << " output");

        JSK_NODELET_INFO("INFO output");
        JSK_NODELET_INFO_STREAM("INFO" << " output");
        JSK_NODELET_INFO_THROTTLE(10.0, "INFO output");
        JSK_NODELET_INFO_STREAM_THROTTLE(10.0, "INFO" << " output");

        JSK_NODELET_WARN("WARN output");
        JSK_NODELET_WARN_STREAM("WARN" << " output");
        JSK_NODELET_WARN_THROTTLE(10.0, "WARN output");
        JSK_NODELET_WARN_STREAM_THROTTLE(10.0, "WARN" << " output");

        JSK_NODELET_ERROR("ERROR output");
        JSK_NODELET_ERROR_STREAM("ERROR" << " output");
        JSK_NODELET_ERROR_THROTTLE(10.0, "ERROR output");
        JSK_NODELET_ERROR_STREAM_THROTTLE(10.0, "ERROR" << " output");

        JSK_NODELET_FATAL("FATAL output");
        JSK_NODELET_FATAL_STREAM("FATAL" << " output");
        JSK_NODELET_FATAL_THROTTLE(10.0, "FATAL output");
        JSK_NODELET_FATAL_STREAM_THROTTLE(10.0, "FATAL" << " output");
      }
  };
}

typedef jsk_topic_tools::LogUtils LogUtils;
PLUGINLIB_EXPORT_CLASS(LogUtils, nodelet::Nodelet)
