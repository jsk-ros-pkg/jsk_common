// -*- mode: C++ -*-
#ifndef __JSK_CREATIVE_GESTURE_ROS__
#define __JSK_CREATIVE_GESTURE_ROS__

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>


#include <DepthSense.hxx>
#include <boost/format.hpp>
#include <boost/thread.hpp>
#include <vector>
#include <limits>
#include <string>

namespace jsk_creative_gesture{
  class CreativeGestureRos{
  public:
    CreativeGestureRos();
    void
    ColorInput(DepthSense::ColorNode obj, DepthSense::ColorNode::NewSampleReceivedData data);
    void
    DepthInput(DepthSense::DepthNode obj, DepthSense::DepthNode::NewSampleReceivedData data);
    void
    DeviceAdded(DepthSense::Context obj, DepthSense::Context::DeviceAddedData data);
    void
    GetDepthDevice();
    void
    RosPublishMainLoop();
    void
    GetRosParam();
    void
    SetDepthNodeConfig();
    void
    SetImageNodeConfig();
    void
    RegisterNodes();
    void
    RosSetUp();
    void
    SetCameraInfo(DepthSense::IntrinsicParameters &intrinsics, DepthSense::ExtrinsicParameters &extrinsics, sensor_msgs::CameraInfo &camera_info);
    void
    Run();

    //sensor related
    DepthSense::DepthNode depth_node_;
    DepthSense::ColorNode color_node_;
    DepthSense::Context   context_;
    DepthSense::StereoCameraParameters stereo_camera_parameters_;
    DepthSense::CompressionType image_compressed_type_;

    //depth config ros params
    int depth_confidence_threshold_;
    int depth_framerate_;
    int depth_frameformat_;
    int depth_mode_;
    bool depth_saturation_;

    //color config ros params
    int color_compression_type_;
    int color_framerate_;
    int color_frameformat_;
    int color_power_line_;

    //ROS node related
    ros::NodeHandle n_;
    ros::Publisher depth_pub_;
    ros::Publisher color_pub_;
    ros::Publisher depim_pub_;
    ros::Publisher depth_camera_pub_;
    ros::Publisher image_camera_pub_;

    int32_t image_width, image_height;
    int32_t depth_width, depth_height;
    std::string frame_id_;

  };
};


#endif
