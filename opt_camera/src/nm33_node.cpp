#include <ros/ros.h>
#include <rospack/rospack.h>
#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <image_transport/image_transport.h>
#include <camera_calibration_parsers/parse.h>

#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>

#include <dynamic_reconfigure/server.h>
#include <opt_camera/OptNM33CameraConfig.h>
#include "opt_nm33_uvc/opt_nm33_camera.h"

class OptCamNode
{
private:
  // camera
  OptNM3xCamera *camera_;
  std::string serial_id_;

  // ros
  ros::NodeHandle &node_;
  ros::Time next_time_;
  int count_;

  // Publications
  sensor_msgs::Image img_, img_omni_, img_wide_, img_middle_, img_narrow_, img_panorama_;
  image_transport::CameraPublisher img_pub_, img_omni_pub_, img_wide_pub_, img_middle_pub_, img_narrow_pub_,img_panorama_pub_;
  sensor_msgs::CameraInfo info_, info_omni_, info_wide_, info_middle_, info_narrow_, info_panorama_;
  ros::ServiceServer info_srv_, info_omni_srv_, info_wide_srv_, info_middle_srv_, info_narrow_srv_, info_panorama_srv_;

  // Dynamic reconfigure
  typedef dynamic_reconfigure::Server<opt_camera::OptNM33CameraConfig> ReconfigureServer;
  ReconfigureServer reconfigure_server_;
  opt_camera::OptNM33CameraConfig config_;
  int pan;

public:
  OptCamNode(ros::NodeHandle &node) : node_(node)
  {
    // camera device
    ros::NodeHandle nh("~");
    int camera_index = 0;

    nh.getParam("camera_index", camera_index);
    camera_ = new OptNM3xCamera(camera_index);
    if ((serial_id_ = camera_->getSerialID()) == "") {
      nh.getParam("serial_id", serial_id_);
    }
    camera_->setSmallHemisphere(1);
    camera_->setLocationAbsolute(0, 0, 0, 0,   0);
    camera_->setLocationAbsolute(1, 0, 0, 0,  40);
    camera_->setLocationAbsolute(2, 0, 0, 0, 120);

    // image header
    img_.header.frame_id = std::string("head_camera");
    img_omni_.header.frame_id = std::string("head_camera");
    img_wide_.header.frame_id = std::string("head_camera");
    img_middle_.header.frame_id = std::string("head_camera");
    img_narrow_.header.frame_id = std::string("head_camera");
    img_panorama_.header.frame_id = std::string("head_camera");
    nh.getParam("frame_id", img_.header.frame_id);
    nh.getParam("omni_frame_id", img_omni_.header.frame_id);
    nh.getParam("wide_frame_id", img_wide_.header.frame_id);
    nh.getParam("middle_frame_id", img_middle_.header.frame_id);
    nh.getParam("narrow_frame_id", img_narrow_.header.frame_id);
    nh.getParam("panorama_frame_id", img_panorama_.header.frame_id);

    // info
    info_.header.frame_id = img_.header.frame_id;
    info_omni_.header.frame_id = img_omni_.header.frame_id;
    info_wide_.header.frame_id = img_wide_.header.frame_id;
    info_middle_.header.frame_id = img_middle_.header.frame_id;
    info_narrow_.header.frame_id = img_narrow_.header.frame_id;
    info_panorama_.header.frame_id = img_panorama_.header.frame_id;

    // advertise
    img_pub_ = image_transport::ImageTransport(node).advertiseCamera("image_raw" , 1);
    img_omni_pub_ = image_transport::ImageTransport(ros::NodeHandle(node,"omni")).advertiseCamera("image_raw" , 1);
    img_wide_pub_ = image_transport::ImageTransport(ros::NodeHandle(node,"wide")).advertiseCamera("image_raw" , 1);
    img_middle_pub_ = image_transport::ImageTransport(ros::NodeHandle(node,"middle")).advertiseCamera("image_raw" , 1);
    img_narrow_pub_ = image_transport::ImageTransport(ros::NodeHandle(node,"narrow")).advertiseCamera("image_raw" , 1);
    img_panorama_pub_ = image_transport::ImageTransport(ros::NodeHandle(node,"panorama")).advertiseCamera("image_raw" , 1);

    info_srv_ = node.advertiseService("set_camera_info", &OptCamNode::set_camera_info, this);
    info_omni_srv_ = ros::NodeHandle(node,"omni").advertiseService("set_camera_info", &OptCamNode::set_camera_info_omni, this);
    info_wide_srv_ = ros::NodeHandle(node,"wide").advertiseService("set_camera_info", &OptCamNode::set_camera_info_wide, this);
    info_middle_srv_ = ros::NodeHandle(node,"middle").advertiseService("set_camera_info", &OptCamNode::set_camera_info_middle, this);
    info_narrow_srv_ = ros::NodeHandle(node,"narrow").advertiseService("set_camera_info", &OptCamNode::set_camera_info_narrow, this);
    info_panorama_srv_ = ros::NodeHandle(node,"panorama").advertiseService("set_camera_info", &OptCamNode::set_camera_info_panorama, this);

    // get camera info
    get_camera_info_method("",info_);
    get_camera_info_method("-omni",info_omni_);
    get_camera_info_method("-wide",info_wide_);
    get_camera_info_method("-middle",info_middle_);
    get_camera_info_method("-narrow",info_narrow_);
    get_camera_info_method("-panorama",info_panorama_);

    // Set up dynamic reconfiguration
    ReconfigureServer::CallbackType f = boost::bind(&OptCamNode::config_cb, this, _1, _2);
    reconfigure_server_.setCallback(f);

    next_time_ = ros::Time::now();
    count_ = 0;
  }

  ~OptCamNode()
  {
  }

  bool take_and_send_image()
  {
    IplImage *frame = NULL;
    ros::Time now;

    if ( ( frame = camera_->queryFrame() ) == NULL ) {
      ROS_ERROR_STREAM("[" << serial_id_ << "] cvQueryFrame");
      return false;
    }
    now = ros::Time::now();
    img_.header.stamp = now;
    img_omni_.header.stamp = now;
    img_wide_.header.stamp = now;
    img_middle_.header.stamp = now;
    img_narrow_.header.stamp = now;
    img_panorama_.header.stamp = now;
    info_.header.stamp = now;
    info_omni_.header.stamp = now;
    info_wide_.header.stamp = now;
    info_middle_.header.stamp = now;
    info_narrow_.header.stamp = now;
    info_panorama_.header.stamp = now;

    try {
      fillImage(img_, "bgr8", frame->height, frame->width, frame->width * frame->nChannels, frame->imageData);
      img_pub_.publish(img_, info_);
      {
        IplImage *tmp_img_ = camera_->queryOmniFrame();
        fillImage(img_omni_, "bgr8", tmp_img_->height, tmp_img_->width, tmp_img_->width * tmp_img_->nChannels, tmp_img_->imageData);
        img_omni_pub_.publish(img_omni_, info_omni_);
      }
      {
        IplImage *tmp_img_ = camera_->queryWideFrame();
        fillImage(img_wide_, "bgr8", tmp_img_->height, tmp_img_->width, tmp_img_->width * tmp_img_->nChannels, tmp_img_->imageData);
        img_wide_pub_.publish(img_wide_, info_wide_);
      }
      {
        IplImage *tmp_img_ = camera_->queryMiddleFrame();
        fillImage(img_middle_, "bgr8", tmp_img_->height, tmp_img_->width, tmp_img_->width * tmp_img_->nChannels, tmp_img_->imageData);
        img_middle_pub_.publish(img_middle_, info_middle_);
      }
      {
        IplImage *tmp_img_ = camera_->queryNarrowFrame();
        fillImage(img_narrow_, "bgr8", tmp_img_->height, tmp_img_->width, tmp_img_->width * tmp_img_->nChannels, tmp_img_->imageData);
        img_narrow_pub_.publish(img_narrow_, info_narrow_);
      }
      if(config_.mode == 2) { // mode = panorama(2)
        IplImage* tmp_img_ = cvCreateImage(cvSize(frame->width*2,frame->height/2),frame->depth,frame->nChannels);
        // lower -> left, upper -> right
        if ( 0 < pan && pan < 180 ) {
          int offset = frame->width * pan / 180;
          cvSetImageROI(frame,cvRect(0,frame->height/2,frame->width,frame->height/2));
          cvSetImageROI(tmp_img_,cvRect(offset,0,frame->width,frame->height/2));
          cvCopy(frame,tmp_img_);
          cvSetImageROI(frame,cvRect(frame->width-offset,0,offset,frame->height/2));
          cvSetImageROI(tmp_img_,cvRect(0,0,offset,frame->height/2));
          cvCopy(frame,tmp_img_);
          cvSetImageROI(frame,cvRect(0,0,frame->width-offset,frame->height/2));
          cvSetImageROI(tmp_img_,cvRect(frame->width+offset,0,frame->width-offset,frame->height/2));
          cvCopy(frame,tmp_img_);
        } else if ( -180 < pan && pan < 0 ) {
          int offset = frame->width * pan / -180;
          cvSetImageROI(frame,cvRect(0,0,frame->width,frame->height/2));
          cvSetImageROI(tmp_img_,cvRect(frame->width-offset,0,frame->width,frame->height/2));
          cvCopy(frame,tmp_img_);
          cvSetImageROI(frame,cvRect(offset,frame->height/2,frame->width-offset,frame->height/2));
          cvSetImageROI(tmp_img_,cvRect(0,0,frame->width-offset,frame->height/2));
          cvCopy(frame,tmp_img_);
          cvSetImageROI(frame,cvRect(0,frame->height/2,offset,frame->height/2));
          cvSetImageROI(tmp_img_,cvRect(frame->width*2-offset,0,offset,frame->height/2));
          cvCopy(frame,tmp_img_);
        } else { // ( pan == 0 )
          cvSetImageROI(frame,cvRect(0,frame->height/2,frame->width,frame->height/2));
          cvSetImageROI(tmp_img_,cvRect(0,0,frame->width,frame->height/2));
          cvCopy(frame,tmp_img_);
          cvSetImageROI(frame,cvRect(0,0,frame->width,frame->height/2));
          cvSetImageROI(tmp_img_,cvRect(frame->width,0,frame->width,frame->height/2));
          cvCopy(frame,tmp_img_);
        }
        fillImage(img_panorama_, "bgr8", tmp_img_->height, tmp_img_->width, tmp_img_->width * tmp_img_->nChannels, tmp_img_->imageData);
        img_panorama_pub_.publish(img_panorama_, info_panorama_);
        cvResetImageROI(frame);
        cvReleaseImage(&tmp_img_);
      }
    }
    catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("Could not convert from opencv to message.");
      }
    return true;
  }

  void config_cb(opt_camera::OptNM33CameraConfig &config, uint32_t level)
  {
#define SET_CAMERA(methodname, paramname)         \
    if (config_.paramname != config.paramname ) { \
      camera_->methodname(config.paramname);      \
    }

    SET_CAMERA(setMode, mode);
    if ( config.autoexposure ) {
      SET_CAMERA(setAutoExposure, autoexposure);
    } else {
      SET_CAMERA(setExposure, exposure);
      SET_CAMERA(setIris, iris);
    }
    if ( config.mode != 4 && config.mode != 0 ) {
      pan = config.pan;
      SET_CAMERA(setPanAbsolute,  pan);
      SET_CAMERA(setTiltAbsolute, tilt);
      SET_CAMERA(setRollAbsolute, roll);
      SET_CAMERA(setZoomAbsolute, zoom);
    }
    if ( config.autoexposure ) {
      SET_CAMERA(setBrightness, brightness);
    }
    SET_CAMERA(setSharpness, sharpness);
    if ( ! config.autowhitebalance ) {
      SET_CAMERA(setWhitebalance, whitebalance);
    } else {
      SET_CAMERA(setAutoWhitebalance, autowhitebalance);
    }
    config.firmwareversion = camera_->getFirmwareVersion();
    SET_CAMERA(setFlipScreen, flipscreen);
    SET_CAMERA(setSmallHemisphere, smallhemisphere);
    SET_CAMERA(setMedianFilter, medianfilter); // ???
    SET_CAMERA(setJpegQuality, jpegquality);
    config.serialid = serial_id_;
    SET_CAMERA(setInfoDisplay, infodisplay);
    if ( config.capturefps != config_.capturefps) {
      SET_CAMERA(setCaptureFPS, capturefps);
    }
    //config.actualfps = camera_->getActualFPS();
    if ( config.lenstype != config_.lenstype) {
      SET_CAMERA(setLensType, lenstype);
    }
    if ( config.mode == 4 ) {
      // right top
      //camera_->setPanAbsolute (1,config.camera1_pan);
      //camera_->setTiltAbsolute(1,config.camera1_tilt);
      //camera_->setRollAbsolute(1,config.camera1_roll);
      //camera_->setZoomAbsolute(1,config.camera1_zoom);
      if ( ( config.camera1_pan  != config_.camera1_pan ) ||
           ( config.camera1_tilt != config_.camera1_tilt ) ||
           ( config.camera1_roll != config_.camera1_roll ) ||
           ( config.camera1_zoom != config_.camera1_zoom ) ) {
        camera_->setLocationAbsolute(1,config.camera1_pan, config.camera1_tilt, config.camera1_roll, config.camera1_zoom);
      }

      // left bottom
      //camera_->setPanAbsolute (2,config.camera2_pan);
      //camera_->setTiltAbsolute(2,config.camera2_tilt);
      //camera_->setRollAbsolute(2,config.camera2_roll);
      //camera_->setZoomAbsolute(2,config.camera2_zoom);w
      if ( ( config.camera2_pan  != config_.camera2_pan ) ||
           ( config.camera2_tilt != config_.camera2_tilt ) ||
           ( config.camera2_roll != config_.camera2_roll ) ||
           ( config.camera2_zoom != config_.camera2_zoom ) ) {
        camera_->setLocationAbsolute(2,config.camera2_pan, config.camera2_tilt, config.camera2_roll, config.camera2_zoom);
      }

      // right bottom
      if ( ( config.camera3_pan  != config_.camera3_pan ) ||
           ( config.camera3_tilt != config_.camera3_tilt ) ||
           ( config.camera3_roll != config_.camera3_roll ) ||
           ( config.camera3_zoom != config_.camera3_zoom ) ) {
        camera_->setLocationAbsolute(0,config.camera3_pan, config.camera3_tilt, config.camera3_roll, config.camera3_zoom);
      }
      //SET_CAMERA(setPanAbsolute,  camera3_pan);
      //SET_CAMERA(setTiltAbsolute, camera3_tilt);
      //SET_CAMERA(setRollAbsolute, camera3_roll);
      //SET_CAMERA(setZoomAbsolute, camera3_zoom);
#if 0
      if ( config.camera3_pan != config_.camera3_pan ) {
        camera_->setPanAbsolute (0,config.camera3_pan);
      }
      if ( config.camera3_tilt != config_.camera3_tilt ) {
        camera_->setTiltAbsolute(0,config.camera3_tilt);
      }
      if ( config.camera3_roll != config_.camera3_roll ) {
        camera_->setRollAbsolute(0,config.camera3_roll);
      }
      if ( config.camera3_zoom != config_.camera3_zoom ) {
        camera_->setZoomAbsolute(0,config.camera3_zoom);
      }
#endif
    }
    config_ = config;
  }

  bool get_camera_info_method(std::string view_name, sensor_msgs::CameraInfo &info) {
    std::string cam_name = serial_id_+view_name;
    std::string ini_name = cam_name+".ini";
    try {
#ifdef ROSPACK_EXPORT
      rospack::ROSPack rp;
      rospack::Package *p = rp.get_pkg("opt_camera");
      if (p!=NULL) ini_name = p->path + "/cfg/" + ini_name;
#else
      rospack::Rospack rp;
      std::vector<std::string> search_path;
      rp.getSearchPathFromEnv(search_path);
      rp.crawl(search_path, 1);
      std::string path;
      if (rp.find("opt_camera",path)==true) ini_name = path + "/cfg" + ini_name;
#endif
    } catch (std::runtime_error &e) {
    }
    if (!camera_calibration_parsers::readCalibration(ini_name, cam_name, info)) {
      return false;
    }
    if ( info.P[0] == 0.0 ) {
      ROS_ERROR_STREAM( "Loading wrong camera_info from " << ini_name );
      return false;
    }
    ROS_ERROR_STREAM( "Loading camera_info from " << ini_name << info.P[0] );
    return true;
  }

#ifdef ROSPACK_EXPORT
#define set_camera_info_method(set_camera_info_method, view_name)       \
  bool set_camera_info_method(sensor_msgs::SetCameraInfo::Request& req, \
                              sensor_msgs::SetCameraInfo::Response& rsp) { \
    ROS_INFO("New camera info received");                               \
    sensor_msgs::CameraInfo &info = req.camera_info;                    \
                                                                        \
    std::string cam_name = serial_id_+view_name;            \
    std::string ini_name = cam_name+".ini";                             \
    rospack::ROSPack rp;                                                \
    try {                                                               \
      rospack::Package *p = rp.get_pkg("opt_camera");                   \
      if (p!=NULL) ini_name = p->path + "/cfg/" + ini_name;             \
    } catch (std::runtime_error &e) {                                   \
    }                                                                   \
    if (!camera_calibration_parsers::writeCalibration(ini_name, cam_name.c_str(), info)) { \
      rsp.status_message = "Error writing camera_info to " + cam_name + ".ini"; \
      rsp.success = false;                                              \
    }                                                                   \
                                                                        \
    rsp.success = true;                                                 \
    return true;                                                        \
  }
#else
#define set_camera_info_method(set_camera_info_method, view_name)       \
  bool set_camera_info_method(sensor_msgs::SetCameraInfo::Request& req, \
                              sensor_msgs::SetCameraInfo::Response& rsp) { \
    ROS_INFO("New camera info received");                               \
    sensor_msgs::CameraInfo &info = req.camera_info;                    \
                                                                        \
    std::string cam_name = serial_id_+view_name;            \
    std::string ini_name = cam_name+".ini";                             \
    rospack::Rospack rp;                                                \
    try {                                                               \
      std::vector<std::string> search_path;                             \
      rp.getSearchPathFromEnv(search_path);                             \
      rp.crawl(search_path, 1);                                         \
      std::string path;                                                 \
      if(rp.find("opt_camera", path)==true) ini_name = path + "/cfg/" + ini_name; \
    } catch (std::runtime_error &e) {                                   \
    }                                                                   \
    if (!camera_calibration_parsers::writeCalibration(ini_name, cam_name.c_str(), info)) { \
      rsp.status_message = "Error writing camera_info to " + cam_name + ".ini"; \
      rsp.success = false;                                              \
    }                                                                   \
                                                                        \
    rsp.success = true;                                                 \
    return true;                                                        \
  }
#endif

  set_camera_info_method(set_camera_info,"")
  set_camera_info_method(set_camera_info_omni,"-omni")
  set_camera_info_method(set_camera_info_wide,"-wide")
  set_camera_info_method(set_camera_info_middle,"-middle")
  set_camera_info_method(set_camera_info_narrow,"-narrow")
  set_camera_info_method(set_camera_info_panorama,"-panorama")

  bool spin()
  {
    while (node_.ok())
      {
        if (take_and_send_image())
          {
            count_++;
            ros::Time now_time = ros::Time::now();
            if (now_time > next_time_) {
              std::cout << count_ << " frames/sec at " << now_time << std::endl;
              count_ = 0;
              next_time_ = next_time_ + ros::Duration(1,0);
            }
          } else {
          ROS_ERROR_STREAM("[" << node_.getNamespace() << "] couldn't take image.");
          usleep(1000000);
        }
        ros::spinOnce();
      }
    return true;
  }

};


int main (int argc, char **argv)
{
  ros::init(argc, argv, "nm33_cam");
  ros::NodeHandle nh("camera");

  OptCamNode camera(nh);

  camera.spin();

  return 0;
}
