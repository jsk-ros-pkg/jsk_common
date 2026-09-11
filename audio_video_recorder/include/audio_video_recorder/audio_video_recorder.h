#include <boost/thread.hpp>
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <jsk_ros1_ros2_compat/compat.h>

// ROS1/ROS2 compatibility: reopen the message namespaces with `using`
// aliases so that `sensor_msgs::Image` and the original ROS1 flat
// ConstPtr typedef spelling `sensor_msgs::ImageConstPtr` /
// `audio_common_msgs::AudioDataConstPtr` (used by the callback
// signatures below, untouched from master) all resolve to the right
// per-version type.
#if ROS_VERSION_MAJOR != 1
#include <sensor_msgs/msg/image.hpp>
#include <audio_common_msgs/msg/audio_data.hpp>
#else
#include "audio_common_msgs/AudioData.h"
#include "sensor_msgs/Image.h"
#endif
JSK_ROS1_ROS2_COMPAT_MSG_ALIAS(sensor_msgs, Image)
JSK_ROS1_ROS2_COMPAT_CONST_PTR_ALIAS(sensor_msgs, Image)
JSK_ROS1_ROS2_COMPAT_MSG_ALIAS(audio_common_msgs, AudioData)
JSK_ROS1_ROS2_COMPAT_CONST_PTR_ALIAS(audio_common_msgs, AudioData)

namespace audio_video_recorder
{
  class AudioVideoRecorder
  {
    public:
      AudioVideoRecorder() {}

      typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, audio_common_msgs::AudioData> ExactSyncPolicy;
      typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, audio_common_msgs::AudioData> ApproximateSyncPolicy;

      void initialize();
      void callbackImage(const sensor_msgs::ImageConstPtr &image_msg);
      void callbackAudio(const audio_common_msgs::AudioDataConstPtr &audio_msg);
      static void callbackPad(GstElement *decodebin, GstPad *pad, gpointer data);
#if ROS_VERSION_MAJOR != 1
      rclcpp::Node::SharedPtr getNode() { return _nh; }
#endif
    protected:
#if ROS_VERSION_MAJOR != 1
      rclcpp::Node::SharedPtr _nh;
#else
      boost::shared_ptr<ros::NodeHandle> _nh;
#endif
      ROS1_ROS2_COMPAT::RosSubscriber<sensor_msgs::Image> _sub_image;
      ROS1_ROS2_COMPAT::RosSubscriber<audio_common_msgs::AudioData> _sub_audio;
      boost::shared_ptr<message_filters::Synchronizer<ExactSyncPolicy> > _sync;
      boost::shared_ptr<message_filters::Synchronizer<ApproximateSyncPolicy> > _async;

      boost::thread _gst_thread;
      GstElement *_pipeline, *_bin, *_mux, *_sink;
      GstElement *_audio_source, *_audio_filter;
      GstElement *_audio_source_queue, *_audio_queue;
      GstElement *_audio_encoder, *_audio_decoder;
      GstElement *_video_source, *_video_filter;
      GstElement *_video_source_queue, *_video_queue;
      GstElement *_video_convert;
      GMainLoop *_loop;
  };
}
