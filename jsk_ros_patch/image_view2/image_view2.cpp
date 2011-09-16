/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/CvBridge.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>

#include <image_view2/ImageMarker2.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <std_msgs/Empty.h>

#include <boost/thread.hpp>
#include <boost/format.hpp>
#include <boost/foreach.hpp>


typedef std::vector<image_view2::ImageMarker2::ConstPtr> V_ImageMarkerMessage;

#define DEFAULT_COLOR  CV_RGB(255,0,0)
#define USER_ROI_COLOR CV_RGB(255,0,0)
#define DEFAULT_CIRCLE_SCALE  20
#define LINE_WIDTH             3

inline CvScalar MsgToRGB(const std_msgs::ColorRGBA &color){
  if(color.a == 0.0 && color.r == 0.0 && color.g == 0.0 && color.b == 0.0)
    return DEFAULT_COLOR;
  else
    return CV_RGB(color.r*255, color.g*255, color.b*255);
}


class ImageView2
{
private:
  image_transport::Subscriber image_sub_;
  ros::Subscriber info_sub_;
  ros::Subscriber marker_sub_;
  std::string marker_topic_;

  image_transport::Publisher image_pub_;

  V_ImageMarkerMessage marker_queue_;
  boost::mutex queue_mutex_;

  sensor_msgs::ImageConstPtr last_msg_;
  sensor_msgs::CameraInfoConstPtr info_msg_;
  sensor_msgs::CvBridge img_bridge_;
  boost::mutex image_mutex_;
  cv::Mat image_, draw_;

  tf::TransformListener tf_listener_;
  image_geometry::PinholeCameraModel cam_model_;
  std::vector<std::string> frame_ids_;
  boost::mutex info_mutex_;

  std::string window_name_;
  boost::format filename_format_;
  int font_;
  static CvRect window_selection_;
  int count_;
  bool blurry_mode;

  ros::Publisher point_pub_;
  ros::Publisher rectangle_pub_;

public:
  ImageView2() : marker_topic_("image_marker"), filename_format_(""), count_(0)
  {
  }
  ImageView2(ros::NodeHandle& nh, const std::string& transport)
    : marker_topic_("image_marker"), filename_format_(""), count_(0)
  {
    std::string camera = nh.resolveName("image");
    std::string camera_info = nh.resolveName("camera_info");
    ros::NodeHandle local_nh("~");
    bool autosize;
    std::string format_string;
    image_transport::ImageTransport it(nh);

    point_pub_ = nh.advertise<geometry_msgs::PointStamped>(camera + "/screenpoint",100);
    rectangle_pub_ = nh.advertise<geometry_msgs::PolygonStamped>(camera + "/screenrectangle",100);

    local_nh.param("window_name", window_name_, std::string("image_view2 [")+camera+std::string("]"));

    local_nh.param("autosize", autosize, false);

    local_nh.param("blurry", blurry_mode, false);

    local_nh.param("filename_format", format_string, std::string("frame%04i.jpg"));
    filename_format_.parse(format_string);

    cvNamedWindow(window_name_.c_str(), autosize ? CV_WINDOW_AUTOSIZE : 0);
    cvSetMouseCallback(window_name_.c_str(), &ImageView2::mouse_cb, this);
    font_ = cv::FONT_HERSHEY_DUPLEX;
    window_selection_.x = window_selection_.y =
      window_selection_.height = window_selection_.width = 0;
    cvStartWindowThread();

    image_sub_ = it.subscribe(camera, 1, &ImageView2::image_cb, this, transport);
    info_sub_ = nh.subscribe(camera_info, 1, &ImageView2::info_cb, this);
    marker_sub_ = nh.subscribe(marker_topic_, 10, &ImageView2::marker_cb, this);

    image_pub_ = it.advertise("image_marked", 1);
  }

  ~ImageView2()
  {
    cvDestroyWindow(window_name_.c_str());
  }

  void marker_cb(const image_view2::ImageMarker2ConstPtr& marker)
  {
    ROS_DEBUG("marker_cb");
    // convert lifetime to duration from Time(0)
    if(marker->lifetime != ros::Duration(0))
      boost::const_pointer_cast<image_view2::ImageMarker2>(marker)->lifetime = (ros::Time::now() - ros::Time(0)) + marker->lifetime;
    boost::mutex::scoped_lock lock(queue_mutex_);

    marker_queue_.push_back(marker);

  }

  void info_cb(const sensor_msgs::CameraInfoConstPtr& msg) {
    ROS_DEBUG("info_cb");
    boost::mutex::scoped_lock lock(info_mutex_);
    info_msg_ = msg;
  }

  void image_cb(const sensor_msgs::ImageConstPtr& msg)
  {
    static ros::Time old_time;
    ROS_DEBUG("image_cb");
    if(old_time.toSec() - ros::Time::now().toSec() > 0) {
      ROS_WARN("TF Cleared for old time");
    }

    if (msg->encoding.find("bayer") != std::string::npos) {
      image_ = cv::Mat(msg->height, msg->width, CV_8UC1,
		       const_cast<uint8_t*>(&msg->data[0]), msg->step);
    } else {
      try {
	image_ = img_bridge_.imgMsgToCv(msg, "bgr8");
      } catch (sensor_msgs::CvBridgeException& e) {
	ROS_ERROR("Unable to convert %s image to bgr8", msg->encoding.c_str());
	return;
      }
    }
    boost::lock_guard<boost::mutex> guard(image_mutex_);
    // Hang on to message pointer for sake of mouse_cb
    last_msg_ = msg;

    static V_ImageMarkerMessage local_queue;
    {
      boost::mutex::scoped_lock lock(queue_mutex_);

      while ( ! marker_queue_.empty() ) {
	// remove marker by namespace and id
	V_ImageMarkerMessage::iterator new_msg = marker_queue_.begin();
	V_ImageMarkerMessage::iterator message_it = local_queue.begin();
	for ( ; message_it < local_queue.end(); ++message_it ) {
	  if((*new_msg)->ns == (*message_it)->ns && (*new_msg)->id == (*message_it)->id)
	    message_it = local_queue.erase(message_it);
	}
	local_queue.push_back(*new_msg);
	marker_queue_.erase(new_msg);
	// if action == REMOVE and id == -1, clear all marker_queue
	if ( (*new_msg)->action == image_view2::ImageMarker2::REMOVE &&
	     (*new_msg)->id == -1 ) {
	  local_queue.clear();
	}
      }
    }

    // check lifetime and remove REMOVE-type marker msg
    for(V_ImageMarkerMessage::iterator it = local_queue.begin(); it < local_queue.end(); it++) {
      if((*it)->action == image_view2::ImageMarker2::REMOVE ||
	 ((*it)->lifetime.toSec() != 0.0 && (*it)->lifetime.toSec() < ros::Time::now().toSec())) {
	it = local_queue.erase(it);
      }
    }

    // Draw Section
    if ( blurry_mode ) {
      draw_ = cv::Mat(image_.size(), image_.type(), CV_RGB(0,0,0));
    } else {
      draw_ = image_;
    }
    if ( !local_queue.empty() )
      {
        V_ImageMarkerMessage::iterator message_it = local_queue.begin();
        V_ImageMarkerMessage::iterator message_end = local_queue.end();
	ROS_DEBUG("markers = %ld", local_queue.size());
        //processMessage;
        for ( ; message_it != message_end; ++message_it )
          {
            image_view2::ImageMarker2::ConstPtr& marker = *message_it;

	    ROS_DEBUG_STREAM("message type = " << marker->type << ", id " << marker->id);

	    // outline colors
	    std::vector<CvScalar> colors;
	    BOOST_FOREACH(std_msgs::ColorRGBA color, marker->outline_colors) {
	      colors.push_back(MsgToRGB(color));
	    }
	    if(colors.size() == 0) colors.push_back(DEFAULT_COLOR);
	    std::vector<CvScalar>::iterator col_it = colors.begin();

            // CIRCLE, LINE_STRIP, LINE_LIST, POLYGON, POINTS
            switch ( marker->type ) {
            case image_view2::ImageMarker2::CIRCLE: {
              cv::Point2d uv = cv::Point2d(marker->position.x, marker->position.y);
	      if ( blurry_mode ) {
		int s0 = LINE_WIDTH;
		CvScalar co = MsgToRGB(marker->outline_color);
		for (int s1 = s0*10; s1 >= s0; s1--) {
		  double m = pow((1.0-((double)(s1 - s0))/(s0*9)),2);
		  cv::circle(draw_, uv,
			     (marker->scale == 0 ? DEFAULT_CIRCLE_SCALE : marker->scale),
			     CV_RGB(co.val[2] * m,co.val[1] * m,co.val[0] * m),
			     s1);
		}
	      } else {
		cv::circle(draw_, uv, (marker->scale == 0 ? DEFAULT_CIRCLE_SCALE : marker->scale), MsgToRGB(marker->outline_color), LINE_WIDTH);
	      }
              break;
            }
            case image_view2::ImageMarker2::LINE_STRIP: {
              cv::Point2d p0, p1;
	      if ( blurry_mode ) {
		int s0 = LINE_WIDTH;
		std::vector<CvScalar>::iterator col_it = colors.begin();
		CvScalar co = (*col_it);
		for (int s1 = s0*10; s1 >= s0; s1--) {
		  double m = pow((1.0-((double)(s1 - s0))/(s0*9)),2);
		  std::vector<geometry_msgs::Point>::const_iterator it = marker->points.begin();
		  std::vector<geometry_msgs::Point>::const_iterator end = marker->points.end();
		  p0 = cv::Point2d(it->x, it->y); it++;
		  for ( ; it!= end; it++ ) {
		    p1 = cv::Point2d(it->x, it->y);
		    cv::line(draw_, p0, p1,
			     CV_RGB(co.val[2] * m,co.val[1] * m,co.val[0] * m),
			     s1);
		    p0 = p1;
		    if(++col_it == colors.end()) col_it = colors.begin();
		  }
		}
	      } else {
		std::vector<geometry_msgs::Point>::const_iterator it = marker->points.begin();
		std::vector<geometry_msgs::Point>::const_iterator end = marker->points.end();
		p0 = cv::Point2d(it->x, it->y); it++;
		for ( ; it!= end; it++ ) {
		  p1 = cv::Point2d(it->x, it->y);
		  cv::line(draw_, p0, p1, *col_it, LINE_WIDTH);
		  p0 = p1;
		  if(++col_it == colors.end()) col_it = colors.begin();
		}
	      }
              break;
            }
            case image_view2::ImageMarker2::LINE_LIST: {
              cv::Point2d p0, p1;
	      if ( blurry_mode ) {
		int s0 = LINE_WIDTH;
		std::vector<CvScalar>::iterator col_it = colors.begin();
		CvScalar co = (*col_it);
		for (int s1 = s0*10; s1 >= s0; s1--) {
		  double m = pow((1.0-((double)(s1 - s0))/(s0*9)),2);
		  std::vector<geometry_msgs::Point>::const_iterator it = marker->points.begin();
		  std::vector<geometry_msgs::Point>::const_iterator end = marker->points.end();
		  for ( ; it!= end; ) {
		    p0 = cv::Point2d(it->x, it->y); it++;
		    if ( it != end ) p1 = cv::Point2d(it->x, it->y);
		    cv::line(draw_, p0, p1, CV_RGB(co.val[2] * m,co.val[1] * m,co.val[0] * m), s1);
		    it++;
		    if(++col_it == colors.end()) col_it = colors.begin();
		  }
		}
	      } else {
		std::vector<geometry_msgs::Point>::const_iterator it = marker->points.begin();
		std::vector<geometry_msgs::Point>::const_iterator end = marker->points.end();
		for ( ; it!= end; ) {
		  p0 = cv::Point2d(it->x, it->y); it++;
		  if ( it != end ) p1 = cv::Point2d(it->x, it->y);
		  cv::line(draw_, p0, p1, *col_it, LINE_WIDTH);
		  it++;
		  if(++col_it == colors.end()) col_it = colors.begin();
		}
	      }
              break;
            }
            case image_view2::ImageMarker2::POLYGON: {
              cv::Point2d p0, p1;
	      if ( blurry_mode ) {
		int s0 = LINE_WIDTH;
		std::vector<CvScalar>::iterator col_it = colors.begin();
		CvScalar co = (*col_it);
		for (int s1 = s0*10; s1 >= s0; s1--) {
		  double m = pow((1.0-((double)(s1 - s0))/(s0*9)),2);
		  std::vector<geometry_msgs::Point>::const_iterator it = marker->points.begin();
		  std::vector<geometry_msgs::Point>::const_iterator end = marker->points.end();
		  p0 = cv::Point2d(it->x, it->y); it++;
		  for ( ; it!= end; it++ ) {
		    p1 = cv::Point2d(it->x, it->y);
		    cv::line(draw_, p0, p1, CV_RGB(co.val[2] * m,co.val[1] * m,co.val[0] * m), s1);
		    p0 = p1;
		    if(++col_it == colors.end()) col_it = colors.begin();
		  }
		  it = marker->points.begin();
		  p1 = cv::Point2d(it->x, it->y);
		  cv::line(draw_, p0, p1, CV_RGB(co.val[2] * m,co.val[1] * m,co.val[0] * m), s1);
		}
	      } else {
		std::vector<geometry_msgs::Point>::const_iterator it = marker->points.begin();
		std::vector<geometry_msgs::Point>::const_iterator end = marker->points.end();
		p0 = cv::Point2d(it->x, it->y); it++;
		for ( ; it!= end; it++ ) {
		  p1 = cv::Point2d(it->x, it->y);
		  cv::line(draw_, p0, p1, *col_it, LINE_WIDTH);
		  p0 = p1;
		  if(++col_it == colors.end()) col_it = colors.begin();
		}
		it = marker->points.begin();
		p1 = cv::Point2d(it->x, it->y);
		cv::line(draw_, p0, p1, *col_it, LINE_WIDTH);
	      }
              break;
            }
            case image_view2::ImageMarker2::POINTS: {
              BOOST_FOREACH(geometry_msgs::Point p, marker->points)  {
                cv::Point2d uv = cv::Point2d(p.x, p.y);
		if ( blurry_mode ) {
		  int s0 = (marker->scale == 0 ? 3 : marker->scale);
		  CvScalar co = (*col_it);
		  for (int s1 = s0*2; s1 >= s0; s1--) {
		    double m = pow((1.0-((double)(s1 - s0))/s0),2);
		    cv::circle(draw_, uv, s1,
			       CV_RGB(co.val[2] * m,co.val[1] * m,co.val[0] * m),
			       -1);
		  }
		} else {
		  cv::circle(draw_, uv, (marker->scale == 0 ? 3 : marker->scale) , *col_it, -1);
		}
		if(++col_it == colors.end()) col_it = colors.begin();
              }
              break;
            }
            case image_view2::ImageMarker2::FRAMES: {
              {
                boost::mutex::scoped_lock lock(info_mutex_);
                if (!info_msg_) {
                  ROS_WARN("[image_view2] camera_info could not found");
                  break;
                }
                cam_model_.fromCameraInfo(info_msg_);
              }
	      static std::map<std::string, int> tf_fail;
              BOOST_FOREACH(std::string frame_id, marker->frames)  {
                tf::StampedTransform transform;
                ros::Time acquisition_time = msg->header.stamp;
                ros::Duration timeout(1.0 / 2); // wait 0.5 sec
                try {
                  tf_listener_.waitForTransform(cam_model_.tfFrame(), frame_id,
                                                acquisition_time, timeout);
                  tf_listener_.lookupTransform(cam_model_.tfFrame(), frame_id,
                                               acquisition_time, transform);
		  tf_fail[frame_id]=0;
                }
                catch (tf::TransformException& ex) {
		  tf_fail[frame_id]++;
		  if ( tf_fail[frame_id] < 5 ) {
		    ROS_ERROR("[image_view2] TF exception:\n%s", ex.what());
		  } else {
		    ROS_DEBUG("[image_view2] TF exception:\n%s", ex.what());
		  }
                  break;
                }
                // center point
                tf::Point pt = transform.getOrigin();
                cv::Point3d pt_cv(pt.x(), pt.y(), pt.z());
                cv::Point2d uv;
                cam_model_.project3dToPixel(pt_cv, uv);

                static const int RADIUS = 3;
		cv::circle(draw_, uv, RADIUS, DEFAULT_COLOR, -1);

                // x, y, z
                cv::Point2d uv0, uv1, uv2;
                tf::Stamped<tf::Point> pin, pout;

                // x
                pin = tf::Stamped<tf::Point>(tf::Point(0.05, 0, 0), acquisition_time, frame_id);
                tf_listener_.transformPoint(cam_model_.tfFrame(), pin, pout);
                cam_model_.project3dToPixel(cv::Point3d(pout.x(), pout.y(), pout.z()), uv0);
                // y
                pin = tf::Stamped<tf::Point>(tf::Point(0, 0.05, 0), acquisition_time, frame_id);
                tf_listener_.transformPoint(cam_model_.tfFrame(), pin, pout);
                cam_model_.project3dToPixel(cv::Point3d(pout.x(), pout.y(), pout.z()), uv1);

                // z
                pin = tf::Stamped<tf::Point>(tf::Point(0, 0, 0.05), acquisition_time, frame_id);
                tf_listener_.transformPoint(cam_model_.tfFrame(), pin, pout);
                cam_model_.project3dToPixel(cv::Point3d(pout.x(), pout.y(), pout.z()), uv2);

		// draw
		if ( blurry_mode ) {
		  int s0 = 2;
		  CvScalar c0 = CV_RGB(255,0,0);
		  CvScalar c1 = CV_RGB(0,255,0);
		  CvScalar c2 = CV_RGB(0,0,255);
		  for (int s1 = s0*10; s1 >= s0; s1--) {
		    double m = pow((1.0-((double)(s1 - s0))/(s0*9)),2);
		    cv::line(draw_, uv, uv0,
			     CV_RGB(c0.val[2] * m,c0.val[1] * m,c0.val[0] * m),
			     s1);
		    cv::line(draw_, uv, uv1,
			     CV_RGB(c1.val[2] * m,c1.val[1] * m,c1.val[0] * m),
			     s1);
		    cv::line(draw_, uv, uv2,
			     CV_RGB(c2.val[2] * m,c2.val[1] * m,c2.val[0] * m),
			     s1);
		  }
		} else {
		  cv::line(draw_, uv, uv0, CV_RGB(255,0,0), 2);
		  cv::line(draw_, uv, uv1, CV_RGB(0,255,0), 2);
		  cv::line(draw_, uv, uv2, CV_RGB(0,0,255), 2);
		}

                // index
		cv::Size text_size;
                int baseline;
                text_size = cv::getTextSize(frame_id.c_str(), font_, 1.0, 1.0, &baseline);
		cv::Point origin = cv::Point(uv.x - text_size.width / 2,
					     uv.y - RADIUS - baseline - 3);
                cv::putText(draw_, frame_id.c_str(), origin, font_, 1.0, DEFAULT_COLOR, 1.5);
              }
              break;
            }
            case image_view2::ImageMarker2::TEXT: {
	      // draw text simply
	      cv::Size text_size;
	      int baseline;
	      text_size = cv::getTextSize(marker->text.c_str(), font_,
					  1.0, 1.0, &baseline);
	      cv::Point origin = cv::Point(marker->position.x - text_size.width/2,
					   marker->position.y - baseline-3);
	      cv::putText(draw_, marker->text.c_str(), origin, font_, 10, DEFAULT_COLOR);
              break;
            }
	    default: {
	      ROS_WARN("Undefined Marker type(%d)", marker->type);
	      break;
	    }
          }
        }
      }
    cv::rectangle(draw_, cv::Point(window_selection_.x, window_selection_.y),
		  cv::Point(window_selection_.x + window_selection_.width,
			    window_selection_.y + window_selection_.height),
		USER_ROI_COLOR, 3, 8, 0);

    if ( blurry_mode ) cv::addWeighted(image_, 0.9, draw_, 1.0, 0.0, image_);
    cv::imshow(window_name_.c_str(), image_);
    cv_bridge::CvImage out_msg;
    out_msg.header   = msg->header;
    out_msg.encoding = "bgr8";
    out_msg.image    = image_;
    image_pub_.publish(out_msg.toImageMsg());
    old_time = ros::Time::now();
  }

  static void mouse_cb(int event, int x, int y, int flags, void* param)
  {
    ImageView2 *iv = (ImageView2*)param;
    static ros::Time left_buttondown_time(0);
    switch (event){
    case CV_EVENT_MOUSEMOVE:
      if ( ( left_buttondown_time.toSec() > 0 &&
             ros::Time::now().toSec() - left_buttondown_time.toSec() ) >= 1.0 ) {
        window_selection_.width  = x - window_selection_.x;
        window_selection_.height = y - window_selection_.y;
      }
      break;
    case CV_EVENT_LBUTTONDOWN:
      left_buttondown_time = ros::Time::now();
      window_selection_.x = x;
      window_selection_.y = y;
      break;
    case CV_EVENT_LBUTTONUP:
      if ( ( ros::Time::now().toSec() - left_buttondown_time.toSec() ) < 1.0 ) {
        geometry_msgs::PointStamped screen_msg;
        screen_msg.point.x = window_selection_.x;
        screen_msg.point.y = window_selection_.y;
        screen_msg.point.z = 0;
        screen_msg.header.stamp = ros::Time::now();
        ROS_INFO("Publish screen point %s (%d %d)", iv->point_pub_.getTopic().c_str(), x, y);
        iv->point_pub_.publish(screen_msg);
      } else {
        geometry_msgs::PolygonStamped screen_msg;
        screen_msg.polygon.points.resize(2);
        screen_msg.polygon.points[0].x = window_selection_.x;
        screen_msg.polygon.points[0].y = window_selection_.y;
        screen_msg.polygon.points[1].x = window_selection_.x + window_selection_.width;
        screen_msg.polygon.points[1].y = window_selection_.y + window_selection_.height;
        screen_msg.header.stamp = ros::Time::now();
        ROS_INFO("Publish rectangle point %s (%d %d %d %d)", iv->rectangle_pub_.getTopic().c_str(), window_selection_.y, window_selection_.y, window_selection_.width, window_selection_.height);
        iv->rectangle_pub_.publish(screen_msg);
      }
      window_selection_.x = window_selection_.y =
        window_selection_.width = window_selection_.height = 0;
      left_buttondown_time.fromSec(0);
      break;
    case CV_EVENT_RBUTTONDOWN:
      boost::lock_guard<boost::mutex> guard(iv->image_mutex_);
      if (!iv->image_.empty()) {
        std::string filename = (iv->filename_format_ % iv->count_).str();
	cv::imwrite(filename.c_str(), iv->image_);
        ROS_INFO("Saved image %s", filename.c_str());
        iv->count_++;
      } else {
        ROS_WARN("Couldn't save image, no data!");
      }
      break;
    }
    return;
  }
};

int main(int argc, char **argv)
{
  //ros::init(argc, argv, "image_view2", ros::init_options::AnonymousName);
  ros::init(argc, argv, "image_view2");
  ros::NodeHandle n;

  if ( n.resolveName("image") == "/image") {
    ROS_WARN("image_view: image has not been remapped! Typical command-line usage:\n"
             "\t$ ./image_view image:=<image topic> [transport]");
  }

  ImageView2 view(n, "raw");

  ros::spin();

  return 0;
}

CvRect ImageView2::window_selection_;
