#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <boost/thread.hpp>

namespace depth_image_proc {

namespace enc = sensor_msgs::image_encodings;

#define SHIFT_SCALE 0.125
#define FOCAL_LENGTH 570.342224121
#define BASELINE 0.075

class UndistortNodelet : public nodelet::Nodelet
{
  // Subscriptions
  boost::shared_ptr<image_transport::ImageTransport> it_;
  image_transport::CameraSubscriber sub_camera_;
  
  // Publications
  boost::mutex connect_mutex_;
  image_transport::Publisher pub_depth_;

  virtual void onInit();

  void connectCb();

  void depthCb(const sensor_msgs::ImageConstPtr& raw_msg,
               const sensor_msgs::CameraInfoConstPtr& inf_msg);
};

void UndistortNodelet::onInit()
{
  ros::NodeHandle& nh = getNodeHandle();
  it_.reset(new image_transport::ImageTransport(nh));

  // Monitor whether anyone is subscribed to the output
  image_transport::SubscriberStatusCallback connect_cb = boost::bind(&UndistortNodelet::connectCb, this);
  // Make sure we don't enter connectCb() between advertising and assigning to pub_depth_
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  pub_depth_ = it_->advertise("image_fitted", 1, connect_cb, connect_cb);
}

// Handles (un)subscribing when clients (un)subscribe
void UndistortNodelet::connectCb()
{
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  if (pub_depth_.getNumSubscribers() == 0)
  {
    sub_camera_.shutdown();
  }
  else if (!sub_camera_)
  {
    sub_camera_ = it_->subscribeCamera("image_raw", 5, &UndistortNodelet::depthCb, this);
  }
}

void UndistortNodelet::depthCb(const sensor_msgs::ImageConstPtr& input_msg,
                                 const sensor_msgs::CameraInfoConstPtr& info_msg)
{
  // Allocate new Image message
  sensor_msgs::ImagePtr depth_msg( new sensor_msgs::Image );
  depth_msg->header   = input_msg->header;
  depth_msg->encoding = input_msg->encoding;
  depth_msg->height   = input_msg->height;
  depth_msg->width    = input_msg->width;
  depth_msg->step     = input_msg->step;
  depth_msg->data.resize( depth_msg->height * depth_msg->step);

  //float bad_point = std::numeric_limits<float>::quiet_NaN ();

  const uint16_t* input_data = reinterpret_cast<const uint16_t*>(&input_msg->data[0]);
  uint16_t* depth_data = reinterpret_cast<uint16_t*>(&depth_msg->data[0]);
  double u_coeff, v_coeff, d_coeff, offset;
  int dist;
  if(!ros::param::get ("/projector_coefficients/u_coeff", u_coeff)){
    u_coeff = 0.0;
  }
  if(!ros::param::get ("/projector_coefficients/v_coeff", v_coeff)){
    v_coeff = 0.0;
  }
  if(!ros::param::get ("/projector_coefficients/d_coeff", d_coeff)){
    d_coeff = 1.0;
  }
  if(!ros::param::get ("/projector_coefficients/offset", offset)){
    offset = 0.0;
  }
  if(!ros::param::get ("/projector_coefficients/distance", dist)){
    dist = 150;
  }

  float cx = info_msg->K[2];
  float cy = info_msg->K[5];

  for (unsigned index = 0; index < depth_msg->height * depth_msg->width; ++index)
  {
    float i = input_data[index];
    float u = cx - index%depth_msg->width;
    float v = cy - index/depth_msg->width;
    float d = FOCAL_LENGTH * BASELINE / (i * 0.001);
    float d_fitted = d_coeff * d + offset + u_coeff*u*u + v_coeff*v*v;
    float i_fitted = (FOCAL_LENGTH * BASELINE / d_fitted) * 1000;

    if ( (0 < i_fitted) && (u*u + v*v < dist*dist)){
      depth_data[index] = i_fitted;
    }else{
      depth_data[index] = 0;
    }
  }

  pub_depth_.publish(depth_msg);
}

} // namespace depth_image_proc

// Register as nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS (depth_image_proc, undistort, depth_image_proc::UndistortNodelet, nodelet::Nodelet);
