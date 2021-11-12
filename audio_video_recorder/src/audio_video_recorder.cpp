// modified work from audio_play/src/audio_play.cpp

#include "audio_video_recorder/audio_video_recorder.h"

#include <boost/thread.hpp>
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>

#include "audio_common_msgs/AudioData.h"
#include "sensor_msgs/Image.h"

namespace audio_video_recorder
{

  void AudioVideoRecorder::initialize()
  {
    GstCaps *audio_caps;
    GstPad *audio_src_pad;
    GstPad *audio_mux_pad, *g_audio_mux_pad;
    GstPad *audio_cb_pad;

    GstCaps *video_caps;
    GstPad *video_src_pad;
    GstPad *video_mux_pad, *g_video_mux_pad;

    int queue_size;
    bool do_timestamp = true;
    std::string file_name;
    std::string file_format;

    // audio
    int audio_channels;
    int audio_depth;
    int audio_sample_rate;
    std::string audio_format;
    std::string audio_sample_format;

    // video
    int video_framerate;
    int video_height;
    int video_width;
    std::string video_encoding;

    // common parameters
    ros::param::param<int>("~queue_size", queue_size, 100);
    ros::param::param<std::string>("~file_name", file_name, "/tmp/test.avi");
    ros::param::param<std::string>("~file_format", file_format, "avi");

    // audio parameters
    ros::param::param<int>("~audio_channels", audio_channels, 1);
    ros::param::param<int>("~audio_depth", audio_depth, 16);
    ros::param::param<int>("~audio_sample_rate", audio_sample_rate, 16000);
    ros::param::param<std::string>("~audio_format", audio_format, "mp3");
    ros::param::param<std::string>("~audio_sample_format", audio_sample_format, "S16LE");

    // video parametes
    ros::param::param<int>("~video_framerate", video_framerate, 30);
    ros::param::param<int>("~video_height", video_height, 480);
    ros::param::param<int>("~video_width", video_width, 640);
    ros::param::param<std::string>("~video_encoding", video_encoding, "RGB");

    _nh.reset (new ros::NodeHandle ("~"));
    _loop = g_main_loop_new(NULL, false);
    _pipeline = gst_pipeline_new("app_pipeline");

    // audio source
    audio_caps = gst_caps_new_simple(
        "audio/x-raw",
        "format", G_TYPE_STRING, audio_sample_format.c_str(),
        "rate", G_TYPE_INT, audio_sample_rate,
        "channels", G_TYPE_INT, audio_channels,
        "width",    G_TYPE_INT, audio_depth,
        "depth",    G_TYPE_INT, audio_depth,
        "signed",   G_TYPE_BOOLEAN, TRUE,
        "layout", G_TYPE_STRING, "interleaved",
        NULL);
    // _audio_source = gst_element_factory_make("audiotestsrc", "audio_source");
    _audio_source = gst_element_factory_make("appsrc", "audio_source");
    g_object_set(G_OBJECT(_audio_source), "do-timestamp", (do_timestamp) ? TRUE : FALSE, NULL);
    g_object_set(G_OBJECT(_audio_source), "format", GST_FORMAT_TIME, NULL);
    g_object_set(G_OBJECT(_audio_source), "is-live", TRUE, NULL);
    gst_bin_add(GST_BIN(_pipeline), _audio_source);

    // audio source queue
    _audio_source_queue = gst_element_factory_make("queue", "audio_source_queue");
    gst_bin_add(GST_BIN(_pipeline), _audio_source_queue);

    // audio queue
    _audio_queue = gst_element_factory_make("queue", "audio_queue");
    gst_bin_add(GST_BIN(_pipeline), _audio_queue);
    audio_src_pad = gst_element_get_static_pad(_audio_queue, "src");

    // video source
    video_caps = gst_caps_new_simple(
        "video/x-raw",
        "format", G_TYPE_STRING, video_encoding.c_str(),
        "width", G_TYPE_INT, video_width,
        "height", G_TYPE_INT, video_height,
        "framerate", GST_TYPE_FRACTION, video_framerate, 1,
        NULL);
    // _video_source = gst_element_factory_make("videotestsrc", "video_source");
    _video_source = gst_element_factory_make("appsrc", "video_source");
    g_object_set(G_OBJECT(_video_source), "caps", video_caps, NULL);
    g_object_set(G_OBJECT(_video_source), "format", GST_FORMAT_TIME, NULL);
    g_object_set(G_OBJECT(_video_source), "do-timestamp", (do_timestamp) ? TRUE : FALSE, NULL);
    g_object_set(G_OBJECT(_video_source), "is-live", TRUE, NULL);
    gst_bin_add(GST_BIN(_pipeline), _video_source);
    gst_caps_unref(video_caps);

    // video source queue
    _video_source_queue = gst_element_factory_make("queue", "video_source_queue");
    gst_bin_add(GST_BIN(_pipeline), _video_source_queue);

    // video convert
    _video_convert = gst_element_factory_make("videoconvert", "video_convert");
    gst_bin_add(GST_BIN(_pipeline), _video_convert);

    // video x264 encoding
    _video_filter = gst_element_factory_make("x264enc", "video_filter");
    g_object_set(G_OBJECT(_video_filter), "tune", 4, NULL);
    gst_bin_add(GST_BIN(_pipeline), _video_filter);
    // video_src_pad = gst_element_get_static_pad(_video_filter, "src");

    // video queue
    _video_queue = gst_element_factory_make("queue", "video_queue");
    gst_bin_add(GST_BIN(_pipeline), _video_queue);
    video_src_pad = gst_element_get_static_pad(_video_queue, "src");

    // mux
    if (file_format == "avi")
    {
      _mux = gst_element_factory_make("avimux", "mux");
      audio_mux_pad = gst_element_get_request_pad(_mux, "audio_%u");
      video_mux_pad = gst_element_get_request_pad(_mux, "video_%u");
    }
    else
    {
      ROS_ERROR("Unsupported file format: %s", file_format.c_str());
    }

    // sink
    _sink = gst_element_factory_make("filesink", "sink");
    g_object_set(G_OBJECT(_sink), "location", file_name.c_str(), NULL);

    // bin
    _bin = gst_bin_new("bin");
    g_object_set(G_OBJECT(_bin), "message-forward", TRUE, NULL);
    gst_bin_add_many(GST_BIN(_bin), _mux, _sink, NULL);
    if (gst_element_link_many(_mux, _sink, NULL) != TRUE)
    {
      ROS_ERROR("failed to link gstreamer");
      return;
    }
    g_audio_mux_pad = gst_ghost_pad_new("audio_sink", audio_mux_pad);
    gst_element_add_pad(_bin, g_audio_mux_pad);
    gst_object_unref(GST_OBJECT(audio_mux_pad));

    g_video_mux_pad = gst_ghost_pad_new("video_sink", video_mux_pad);
    gst_element_add_pad(_bin, g_video_mux_pad);
    gst_object_unref(GST_OBJECT(video_mux_pad));

    gst_element_sync_state_with_parent(_bin);
    gst_bin_add(GST_BIN(_pipeline), _bin);

    ROS_INFO("Saving file to %s", file_name.c_str());
    if (audio_format == "mp3")
    {
      // audio decode
      _audio_decoder = gst_element_factory_make("decodebin", "audio_decoder");
      g_signal_connect(_audio_decoder, "pad-added", G_CALLBACK(AudioVideoRecorder::callbackPad), this);
      gst_bin_add(GST_BIN(_pipeline), _audio_decoder);

      // audio caps filter
      _audio_filter = gst_element_factory_make("capsfilter", "audio_filter");
      g_object_set(G_OBJECT(_audio_filter), "caps", audio_caps, NULL);
      gst_bin_add(GST_BIN(_pipeline), _audio_filter);
      // audio_src_pad = gst_element_get_static_pad(_audio_filter, "src");

      if (gst_element_link_many(_video_source, _video_source_queue, _video_convert, _video_filter, _video_queue, NULL) != TRUE ||
          gst_pad_link(video_src_pad, g_video_mux_pad) != GST_PAD_LINK_OK ||
          gst_element_link_many(_audio_source, _audio_source_queue, _audio_decoder, NULL) != TRUE ||
          gst_element_link_many(_audio_filter, _audio_queue, NULL) != TRUE ||
          gst_pad_link(audio_src_pad, g_audio_mux_pad) != GST_PAD_LINK_OK)
      {
        ROS_ERROR("failed to link mp3");
        ROS_ERROR("failed to link gstreamer");
        return;
      }
      else
      {
        ROS_INFO("succeeded to link mp3");
      }
      gst_caps_unref(audio_caps);
    }
    else if (audio_format == "wave")
    {
      g_object_set(G_OBJECT(_audio_source), "caps", audio_caps, NULL);

      // audio wave encoding
      _audio_encoder = gst_element_factory_make("wavenc", "audio_encoder");
      gst_bin_add(GST_BIN(_pipeline), _audio_encoder);
      // audio wave parse
      _audio_decoder = gst_element_factory_make("wavparse", "audio_parser");
      gst_bin_add(GST_BIN(_pipeline), _audio_decoder);
      // audio_src_pad = gst_element_get_static_pad(_audio_decoder, "src");

      if (gst_element_link_many(_video_source, _video_source_queue, _video_convert, _video_filter, _video_queue, NULL) != TRUE ||
          gst_pad_link(video_src_pad, g_video_mux_pad) != GST_PAD_LINK_OK ||
          gst_element_link_many(_audio_source, _audio_source_queue, _audio_encoder, _audio_decoder, _audio_queue, NULL) != TRUE ||
          gst_pad_link(audio_src_pad, g_audio_mux_pad) != GST_PAD_LINK_OK)
      {
        ROS_ERROR("failed to link wave");
        ROS_ERROR("failed to link gstreamer");
        return;
      }
      else
      {
        ROS_INFO("succeeded to link wave");
      }
      gst_caps_unref(audio_caps);
    }
    else
    {
      ROS_ERROR("Unsupported audio format: %s", audio_format.c_str());
    }
    gst_element_set_state(GST_ELEMENT(_pipeline), GST_STATE_PLAYING);
    _gst_thread = boost::thread( boost::bind(g_main_loop_run, _loop));
    _sub_image = _nh->subscribe("input/image", queue_size, &AudioVideoRecorder::callbackImage, this);
    _sub_audio = _nh->subscribe("input/audio", queue_size, &AudioVideoRecorder::callbackAudio, this);
  }

  void AudioVideoRecorder::callbackImage(const sensor_msgs::ImageConstPtr &image_msg)
  {
    GstFlowReturn ret;
    GstBuffer *image_buffer = gst_buffer_new_and_alloc(image_msg->data.size());
    gst_buffer_fill(image_buffer, 0, &image_msg->data[0], image_msg->data.size());
    g_signal_emit_by_name(_video_source, "push-buffer", image_buffer, &ret);
    return;
  }

  void AudioVideoRecorder::callbackAudio(const audio_common_msgs::AudioDataConstPtr &audio_msg)
  {
    GstFlowReturn ret;
    GstBuffer *audio_buffer = gst_buffer_new_and_alloc(audio_msg->data.size());
    gst_buffer_fill(audio_buffer, 0, &audio_msg->data[0], audio_msg->data.size());
    g_signal_emit_by_name(_audio_source, "push-buffer", audio_buffer, &ret);
    return;
  }

  void AudioVideoRecorder::callbackPad(GstElement *decodebin, GstPad *pad, gpointer data)
  {
    AudioVideoRecorder *client = reinterpret_cast<AudioVideoRecorder*>(data);
    GstCaps *caps;
    GstStructure *str;
    GstPad *audiopad;

    /* only link once */
    audiopad = gst_element_get_static_pad (client->_audio_filter, "sink");
    if (GST_PAD_IS_LINKED (audiopad))
    {
      ROS_ERROR("failed to get audio pad sink");
      g_object_unref (audiopad);
      return;
    }

    /* check media type */
    caps = gst_pad_query_caps (pad, NULL);
    str = gst_caps_get_structure (caps, 0);
    if (!g_strrstr (gst_structure_get_name (str), "audio")) {
      ROS_ERROR("failed to get media type");
      gst_caps_unref (caps);
      gst_object_unref (audiopad);
      return;
    }

    gst_caps_unref (caps);
    gst_pad_link (pad, audiopad);
    g_object_unref (audiopad);
    return;
  }
}


int main (int argc, char **argv)
{
  ros::init(argc, argv, "audio_video_recorder");
  gst_init(&argc, &argv);

  audio_video_recorder::AudioVideoRecorder client;
  client.initialize();
  ros::spin();
}
