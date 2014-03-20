#include <jsk_creative_gesture_launch/jsk_creative_gesture_ros.h>

namespace jsk_creative_gesture{
  CreativeGestureRos::CreativeGestureRos():n_(ros::NodeHandle("~"))
  {
    ROS_INFO("Initialize CreativeGestureRos");

    //=========================================================
    // ROS Set UP
    //=========================================================
    RosSetUp();

    //=========================================================
    // DepthSense Set UP
    //=========================================================
    //create connection
    context_ = DepthSense::Context::create("localhost");

    //confirm device existence and get nodes
    GetDepthDevice();

    //Set callbacks and do settings
    SetDepthNodeConfig();
    SetImageNodeConfig();

    //register nodes
    RegisterNodes();

  };

  void
  CreativeGestureRos::DeviceAdded(DepthSense::Context obj, DepthSense::Context::DeviceAddedData data){
    ROS_INFO("Added");
  };

  void CreativeGestureRos::GetDepthDevice(){
    // //Set Callback
    // context_.deviceAddedEvent().connect<CreativeGestureRos>(this, &CreativeGestureRos::DeviceAdded);
    bool get_nodes = false;
    while(!get_nodes){
      std::vector<DepthSense::Device> devices = context_.getDevices();
      if(devices.size() > 0){
        ROS_INFO("Detected Device Num : %ld", devices.size());
        for (int i = 0; i < devices.size(); i++)
          {
            std::vector<DepthSense::Node> nodes = devices[i].getNodes();
            for (int j = 0; j < nodes.size(); j++)
              {
                DepthSense::Node node = nodes[j];
                if (!color_node_.isSet() && node.is<DepthSense::ColorNode>())
                  color_node_ = node.as<DepthSense::ColorNode>();
                if (!depth_node_.isSet() && node.is<DepthSense::DepthNode>())
                  depth_node_ = node.as<DepthSense::DepthNode>();

                if(color_node_.isSet() && depth_node_.isSet()){
                  get_nodes = true;
                  ROS_INFO("Success open NODE!");
                }
              }
            if(get_nodes){
              stereo_camera_parameters_ = devices[i].getStereoCameraParameters();
              break;
            }
            else
              ROS_ERROR("Couldn't get NODES");
          }
      }

      if(!get_nodes){
        ROS_ERROR("No available device detected ...");
        exit(-1);
      }
    }
    ROS_INFO("Detected Device !");
}

  void CreativeGestureRos::GetRosParam(){

    //Get depth confidence
    n_.param("depth_confidence_threshold", depth_confidence_threshold_, 100);
    n_.param("depth_frameformat", depth_frameformat_, 4);   //default QVGA
    n_.param("depth_framerate", depth_framerate_, 30);
    n_.param("depth_mode", depth_mode_, 0);                 //default CLOSE_MODE
    n_.param("depth_saturation", depth_saturation_, true);

    //Get color confidence
    n_.param("color_compression_type", color_compression_type_, 1); //default type_mjpeg
    n_.param("color_frameformat", color_frameformat_, 7); //default VGA
    n_.param("color_framerate", color_framerate_, 30);
    n_.param("color_power_line", color_power_line_, 1); //default POWER_LINE_FREQUENCY_50HZ
  }

  void CreativeGestureRos::SetDepthNodeConfig()
  {
    context_.requestControl(depth_node_);

    depth_node_.setEnableDepthMap(true);
    depth_node_.setEnableDepthMapFloatingPoint(true);
    depth_node_.setEnableVerticesFloatingPoint(true);
    depth_node_.setConfidenceThreshold(depth_confidence_threshold_);

    DepthSense::DepthNode::Configuration depth_config = depth_node_.getConfiguration();
    depth_config.frameFormat = (DepthSense::FrameFormat)depth_frameformat_;
    depth_config.framerate = depth_framerate_;
    depth_config.mode = (DepthSense::DepthNode::CameraMode)depth_mode_;
    depth_config.saturation = depth_saturation_;

    depth_node_.setConfiguration(depth_config);

    DepthSense::FrameFormat_toResolution(depth_config.frameFormat, &depth_width, &depth_height);

    context_.releaseControl(depth_node_);
    depth_node_.newSampleReceivedEvent().connect<CreativeGestureRos>(this, &CreativeGestureRos::DepthInput);
  }

  void CreativeGestureRos::SetImageNodeConfig()
  {
    context_.requestControl(color_node_);

    color_node_.setEnableColorMap(true);

    DepthSense::ColorNode::Configuration color_config = color_node_.getConfiguration();
    color_config.compression = (DepthSense::CompressionType)color_compression_type_;
    color_config.frameFormat = (DepthSense::FrameFormat)color_frameformat_;
    color_config.framerate = color_framerate_;
    color_config.powerLineFrequency = (DepthSense::PowerLineFrequency)color_power_line_;

    DepthSense::FrameFormat_toResolution(color_config.frameFormat, &image_width, &image_height);
    image_compressed_type_ = color_config.compression;
    color_node_.setConfiguration(color_config);
    context_.releaseControl(color_node_);

    color_node_.newSampleReceivedEvent().connect<CreativeGestureRos>(this, &CreativeGestureRos::ColorInput);
  }

  void CreativeGestureRos::RegisterNodes()
  {
    context_.registerNode(color_node_);
    context_.registerNode(depth_node_);
  }

  void CreativeGestureRos::RosSetUp()
  {
    depth_pub_ = n_.advertise<sensor_msgs::PointCloud2>("depth/points", 1);
    color_pub_ = n_.advertise<sensor_msgs::Image>("rgb/image_rect_color", 1);
    depim_pub_ = n_.advertise<sensor_msgs::Image>("depth/image_rect", 1);
    depth_camera_pub_ = n_.advertise<sensor_msgs::CameraInfo>("depth/camera_info", 1);
    image_camera_pub_ = n_.advertise<sensor_msgs::CameraInfo>("rgb/camera_info", 1);

    frame_id_ = "gesture_camera";

    GetRosParam();
  }

  void CreativeGestureRos::SetCameraInfo(DepthSense::IntrinsicParameters &intrinsics, DepthSense::ExtrinsicParameters &extrinsics, sensor_msgs::CameraInfo &camera_info)
  {
    camera_info.header.stamp = ros::Time::now();
    camera_info.header.frame_id = frame_id_;

    camera_info.width = intrinsics.width;
    camera_info.height = intrinsics.height;
    camera_info.distortion_model = "plumb_bob";

    camera_info.D.push_back(intrinsics.k1);
    camera_info.D.push_back(intrinsics.k2);
    camera_info.D.push_back(extrinsics.t1);
    camera_info.D.push_back(extrinsics.t2);
    camera_info.D.push_back(intrinsics.k3);

    camera_info.K[0] = intrinsics.fx;
    camera_info.K[1] = 0;
    camera_info.K[2] = intrinsics.cx;
    camera_info.K[3] = 0;
    camera_info.K[4] = intrinsics.fy;
    camera_info.K[5] = intrinsics.cy;
    camera_info.K[6] = 0;
    camera_info.K[7] = 0;
    camera_info.K[8] = 1;

    camera_info.R[0] = extrinsics.r11;
    camera_info.R[1] = extrinsics.r12;
    camera_info.R[2] = extrinsics.r13;
    camera_info.R[3] = extrinsics.r21;
    camera_info.R[4] = extrinsics.r22;
    camera_info.R[5] = extrinsics.r23;
    camera_info.R[6] = extrinsics.r31;
    camera_info.R[7] = extrinsics.r32;
    camera_info.R[8] = extrinsics.r33;

    camera_info.P[0] = intrinsics.fx;
    camera_info.P[1] = 0;
    camera_info.P[2] = intrinsics.cx;
    camera_info.P[3] = 0;
    camera_info.P[4] = 0;
    camera_info.P[5] = intrinsics.fy;
    camera_info.P[6] = intrinsics.cy;
    camera_info.P[7] = 0;
    camera_info.P[8] = 0;
    camera_info.P[9] = 0;
    camera_info.P[10] = 1;
    camera_info.P[11] = 0;
  }


  void
  CreativeGestureRos::RosPublishMainLoop(){
    ros::Rate loop_rate(30);
    while(ros::ok()){
      ros::spinOnce();

      DepthSense::IntrinsicParameters colorIntrinsics = stereo_camera_parameters_.colorIntrinsics;
      DepthSense::IntrinsicParameters depthIntrinsics = stereo_camera_parameters_.depthIntrinsics;
      DepthSense::ExtrinsicParameters extrinsics = stereo_camera_parameters_.extrinsics;

      sensor_msgs::CameraInfo sensor_depth_camera_info;
      SetCameraInfo(depthIntrinsics, extrinsics, sensor_depth_camera_info);
      depth_camera_pub_.publish(sensor_depth_camera_info);

      sensor_msgs::CameraInfo sensor_image_camera_info;
      SetCameraInfo(colorIntrinsics, extrinsics, sensor_image_camera_info);
      image_camera_pub_.publish(sensor_image_camera_info);

      loop_rate.sleep();
    }

    //When Ros Die, kill depth sense
    context_.stopNodes();
    context_.quit();
  }

  void
  CreativeGestureRos::ColorInput(DepthSense::ColorNode obj, DepthSense::ColorNode::NewSampleReceivedData data){
    sensor_msgs::Image sensor_image;
    sensor_image.header.stamp = ros::Time::now();
    sensor_image.header.frame_id = frame_id_;

    sensor_image.height = image_height;
    sensor_image.width = image_width;
    if(image_compressed_type_ == DepthSense::COMPRESSION_TYPE_MJPEG){
      sensor_image.step = image_width * 3;
      sensor_image.encoding = "bgr8";
      for(int index = 0 ; index < sensor_image.step * image_height; index++)
        sensor_image.data.push_back(data.colorMap[index]);
    }
    else{
      sensor_image.step = image_width * 2;
      sensor_image.encoding = "yuv422";
      sensor_image.data.resize(data.colorMap.size());
      for(int index = 0; index < data.colorMap.size(); index+=4){
        uint8_t y0 = data.colorMap[index];
        uint8_t u0 = data.colorMap[index+1];
        uint8_t y1 = data.colorMap[index+2];
        uint8_t v0 = data.colorMap[index+3];
        memcpy(&sensor_image.data[index*sizeof(uint8_t) + 0], &u0, sizeof(uint8_t));
        memcpy(&sensor_image.data[index*sizeof(uint8_t) + 1], &y0, sizeof(uint8_t));
        memcpy(&sensor_image.data[index*sizeof(uint8_t) + 2], &v0, sizeof(uint8_t));
        memcpy(&sensor_image.data[index*sizeof(uint8_t) + 3], &y1, sizeof(uint8_t));
      }
    }
    color_pub_.publish(sensor_image);
  }

  void
  CreativeGestureRos::DepthInput(DepthSense::DepthNode obj, DepthSense::DepthNode::NewSampleReceivedData data){
    if(depim_pub_.getNumSubscribers()){
      sensor_msgs::Image sensor_depth;
      sensor_depth.header.stamp = ros::Time::now();
      sensor_depth.header.frame_id = frame_id_;

      sensor_depth.height = depth_height;
      sensor_depth.width = depth_width;
      sensor_depth.encoding = "32FC1";

      sensor_depth.step = depth_width * 4;
      sensor_depth.data.resize(sensor_depth.step * depth_height);
      for(int index = 0 ; index < depth_width * depth_height; index++){
        float tmp_depth = data.depthMapFloatingPoint[index];
        if(tmp_depth < 0.0)
          tmp_depth = std::numeric_limits<float>::quiet_NaN();
        memcpy(&sensor_depth.data[index*sizeof(float)], &tmp_depth, sizeof(float));
      }
      depim_pub_.publish(sensor_depth);
    }

    if(depth_pub_.getNumSubscribers()){
      sensor_msgs::PointCloud2 sensor_pc2;
      sensor_pc2.header.stamp = ros::Time::now();
      sensor_pc2.header.frame_id = frame_id_;

      sensor_pc2.width = depth_width;
      sensor_pc2.height = depth_height;
      sensor_pc2.fields.resize(3);
      sensor_pc2.fields[0].name = "x";
      sensor_pc2.fields[1].name = "y";
      sensor_pc2.fields[2].name = "z";
      sensor_pc2.fields[0].offset = 0;
      sensor_pc2.fields[1].offset = 4;
      sensor_pc2.fields[2].offset = 8;
      sensor_pc2.fields[0].count = sensor_pc2.fields[1].count = sensor_pc2.fields[2].count = 1;
      sensor_pc2.fields[0].datatype = sensor_pc2.fields[1].datatype = sensor_pc2.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
      sensor_pc2.point_step = 12;
      sensor_pc2.row_step = sensor_pc2.point_step * sensor_pc2.width;
      sensor_pc2.is_dense = true;

      sensor_pc2.data.resize(3 * sizeof(float) * depth_width * depth_height);
      for(int index = 0; index < data.verticesFloatingPoint.size(); index++){
        float x = data.verticesFloatingPoint[index].x;
        float y = -data.verticesFloatingPoint[index].y;
        float z = data.verticesFloatingPoint[index].z;
        memcpy(&sensor_pc2.data[index*3*sizeof(float)], &x, sizeof(float));
        memcpy(&sensor_pc2.data[(index*3+1)*sizeof(float)], &y, sizeof(float));
        memcpy(&sensor_pc2.data[(index*3+2)*sizeof(float)], &z, sizeof(float));
      }
      depth_pub_.publish(sensor_pc2);
    }
  }

  void CreativeGestureRos::Run(){
    boost::thread thread_camera_param(boost::bind(&CreativeGestureRos::RosPublishMainLoop, this));
    context_.startNodes();
    context_.run();
  }
};
