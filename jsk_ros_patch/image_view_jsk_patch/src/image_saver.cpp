#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <ros/ros.h>
#include <cv_bridge/CvBridge.h>
#include <boost/format.hpp>

#include "image_view_jsk_patch/SaveImage.h"

sensor_msgs::CvBridge g_bridge;
int g_count = 0;
boost::format g_format;
sensor_msgs::ImageConstPtr last_img;
std::string ff_param, dn_param, sai_param;
bool save_all_img;
std::string format_string;
std::string dir_string;

void save_img(const sensor_msgs::ImageConstPtr& image, std::string fname="", std::string dir="", std::string enc=""){
  std::string filename;
  if (dir == ""){
    ros::param::get(dn_param, dir_string);
    dir = dir_string;
  }
  if (fname == ""){
    ros::param::get(ff_param, format_string);
    fname = format_string;
    if (dir == ""){
      g_format.parse(fname);
    } else {
      g_format.parse(dir + "/" + fname);
    }
    filename = (g_format % g_count).str();
  } else {
    filename = dir + "/" + fname;
  }

  if (enc==""){
    enc = image->encoding;
  }

  if (g_bridge.fromImage(*image,enc)) {
    IplImage *image = g_bridge.toIpl();
    if (image) {
      cvSaveImage(filename.c_str(), image);
      ROS_INFO("Saved image %s", filename.c_str());
      g_count++;
    } else {
      ROS_WARN("Couldn't save image, no data!");
    }
  }
  else
    ROS_ERROR("Unable to convert %s image to bgr8", image->encoding.c_str());
}

void callback(const sensor_msgs::ImageConstPtr& image)
{
  ros::param::get(sai_param, save_all_img);
  
  if (save_all_img){
    save_img(image);
  }
  last_img = image;
  return;
}

bool srv_cb(image_view_jsk_patch::SaveImage::Request &req,
            image_view_jsk_patch::SaveImage::Response &res){
  if(last_img){
    save_img(last_img, req.filename, req.directory, req.encoding);
    res.result = true;
    return true;
  }
  res.result = false;
  return false;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_saver", ros::init_options::AnonymousName);
  ros::NodeHandle nh;
  
  ff_param = nh.resolveName("filename_format");
  dn_param = nh.resolveName("directory_name");
  sai_param = nh.resolveName("save_all_subscribed_image");

  nh.param(ff_param, format_string, std::string("frame%04i.jpg"));
  nh.param(dn_param, dir_string, std::string(""));
  nh.param(sai_param, save_all_img, false);
  
  ros::Subscriber sub = nh.subscribe(nh.resolveName("image"), 1, &callback);

  ros::ServiceServer service = nh.advertiseService ("save_image", srv_cb);

  ros::spin();
}
