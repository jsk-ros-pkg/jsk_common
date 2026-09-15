// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, JSK Lab
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

#ifndef IMAGE_VIEW2_H_
#define IMAGE_VIEW2_H_

// ROS1/ROS2 detection (ROS_VERSION_MAJOR) and the shared compat layer
// (RosTime/RosPublisher<T>/ROS_INFO macros/JSK_ROS1_ROS2_COMPAT_MSG_ALIAS/
// ...) come from jsk_ros1_ros2_compat/compat.h, included first since
// the message-type aliasing below depends on it. jsk_ros1_ros2_compat/
// const_ptr.h (the ConstPtr->ConstSharedPtr shim) is included
// separately, further below, after this file's own message headers --
// see that header for why.
#include <jsk_ros1_ros2_compat/compat.h>

#if ROS_VERSION_MAJOR != 1
// <rclcpp/rclcpp.hpp> already included by compat.h above.
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <image_transport/image_transport.hpp>
#include <image_geometry/pinhole_camera_model.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <image_view2/msg/image_marker2.hpp>
#include <image_view2/msg/point_array_stamped.hpp>
#include <image_view2/msg/mouse_event.hpp>
#include <image_view2/srv/change_mode.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_srvs/srv/empty.hpp>
#include <boost/thread.hpp>
#include <boost/format.hpp>
#include <boost/foreach.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/lambda/lambda.hpp>

// ROS1/ROS2 compatibility shim: rather than touching every one of the
// ~70 call sites below that spell out ROS1's generated typedefs
// (image_view2::ImageMarker2::ConstPtr, sensor_msgs::ImageConstPtr,
// etc.), alias them onto their ROS2 equivalents here.
// JSK_ROS1_ROS2_COMPAT_MSG_ALIAS()/_CONST_PTR_ALIAS()/
// _SRV_REQ_RES_ALIAS() (jsk_ros1_ros2_compat/msg_alias.h) cover the
// flat-name `Type`/`TypeConstPtr`/`TypeRequest`/`TypeResponse`
// patterns generically.
JSK_ROS1_ROS2_COMPAT_MSG_ALIAS(image_view2, ImageMarker2)
JSK_ROS1_ROS2_COMPAT_CONST_PTR_ALIAS(image_view2, ImageMarker2)
JSK_ROS1_ROS2_COMPAT_MSG_ALIAS(image_view2, PointArrayStamped)
JSK_ROS1_ROS2_COMPAT_MSG_ALIAS(image_view2, MouseEvent)
JSK_ROS1_ROS2_COMPAT_SRV_ALIAS(image_view2, ChangeMode)
JSK_ROS1_ROS2_COMPAT_SRV_REQ_RES_ALIAS(image_view2, ChangeMode)
JSK_ROS1_ROS2_COMPAT_MSG_ALIAS(sensor_msgs, Image)
JSK_ROS1_ROS2_COMPAT_CONST_PTR_ALIAS(sensor_msgs, Image)
JSK_ROS1_ROS2_COMPAT_MSG_ALIAS(sensor_msgs, CameraInfo)
JSK_ROS1_ROS2_COMPAT_CONST_PTR_ALIAS(sensor_msgs, CameraInfo)
JSK_ROS1_ROS2_COMPAT_MSG_ALIAS(sensor_msgs, PointCloud2)
JSK_ROS1_ROS2_COMPAT_SRV_ALIAS(std_srvs, Empty)
JSK_ROS1_ROS2_COMPAT_SRV_REQ_RES_ALIAS(std_srvs, Empty)
JSK_ROS1_ROS2_COMPAT_MSG_ALIAS(geometry_msgs, Point)
JSK_ROS1_ROS2_COMPAT_MSG_ALIAS(geometry_msgs, Point32)
JSK_ROS1_ROS2_COMPAT_MSG_ALIAS(geometry_msgs, PointStamped)
JSK_ROS1_ROS2_COMPAT_MSG_ALIAS(geometry_msgs, PolygonStamped)
JSK_ROS1_ROS2_COMPAT_MSG_ALIAS(geometry_msgs, PoseStamped)
JSK_ROS1_ROS2_COMPAT_MSG_ALIAS(geometry_msgs, TransformStamped)
JSK_ROS1_ROS2_COMPAT_MSG_ALIAS(std_msgs, ColorRGBA)
JSK_ROS1_ROS2_COMPAT_MSG_ALIAS(std_msgs, Header)
// Last: only safe now that every message/service header above is
// already fully parsed -- see const_ptr.h.
#include <jsk_ros1_ros2_compat/const_ptr.h>
#else
// <ros/ros.h> already included by the detection block above.
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
// tf2_ros (not tf1) is used on the ROS1 side too, so that the tf lookup
// code below (lookupTransformation() and the draw*3D methods) is
// shared verbatim between ROS1 and ROS2 -- tf2_ros::Buffer/
// TransformListener and tf2::doTransform() have the same API on both.
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <dynamic_reconfigure/server.h>

#include <image_view2/ImageMarker2.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <image_view2/ChangeMode.h>
#include <boost/thread.hpp>
#include <boost/format.hpp>
#include <boost/foreach.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/lambda/lambda.hpp>
#include <pcl/point_types.h>
#include <pcl_ros/publisher.h>
#include <image_view2/ImageView2Config.h>

#include <image_view2/MouseEvent.h>
#endif

#if ( CV_MAJOR_VERSION >= 4)
#include <opencv2/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>
#undef CV_RGB
#define CV_RGB( r, g, b )  cvScalar( (b), (g), (r), 0 )
#else
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#endif

#define DEFAULT_COLOR  CV_RGB(255,0,0)
#define USER_ROI_COLOR CV_RGB(255,0,0)
#define DEFAULT_CIRCLE_SCALE  20
#define DEFAULT_LINE_WIDTH    3

namespace image_view2
{
  typedef std::vector<image_view2::ImageMarker2::ConstPtr> V_ImageMarkerMessage;
  inline CvScalar MsgToRGB(const std_msgs::ColorRGBA &color){
    if(color.a == 0.0 && color.r == 0.0 && color.g == 0.0 && color.b == 0.0)
      return DEFAULT_COLOR;
    else
      return CV_RGB(color.r*255, color.g*255, color.b*255);
  }

  class ImageView2
  {
  public:
#if ROS_VERSION_MAJOR == 1
    typedef ImageView2Config Config;
#endif
    enum KEY_MODE {
      MODE_RECTANGLE,
      MODE_SERIES,
      MODE_SELECT_FORE_AND_BACK,
      MODE_SELECT_FORE_AND_BACK_RECT,
      MODE_LINE,
      MODE_POLY,
      MODE_NONE
    };

    ImageView2();
#if ROS_VERSION_MAJOR != 1
    ImageView2(rclcpp::Node::SharedPtr node);
#else
    ImageView2(ros::NodeHandle& nh);
#endif
    ~ImageView2();
    void pressKey(int key);
    void markerCb(const image_view2::ImageMarker2ConstPtr& marker);
    void infoCb(const sensor_msgs::CameraInfoConstPtr& msg);
    void redraw();
    void imageCb(const sensor_msgs::ImageConstPtr& msg);
    void drawImage();
    void addPoint(int x, int y);
    void addRegionPoint(int x, int y);
    void updateRegionWindowSize(int x, int y);
    void setRegionWindowPoint(int x, int y);
    void clearPointArray();
    void publishPointArray();
    void setMode(KEY_MODE mode);
    KEY_MODE getMode();
    void showImage();
    static void mouseCb(int event, int x, int y, int flags, void* param);
    bool isValidMovement(const cv::Point2f& start_point,
                         const cv::Point2f& end_point);
    bool toggleSelection();
    void publishForegroundBackgroundMask();
    bool use_window;
  protected:
  private:
#if ROS_VERSION_MAJOR != 1
    rcl_interfaces::msg::SetParametersResult config_callback(
      const std::vector<rclcpp::Parameter> &parameters);
#else
    void config_callback(Config &config, uint32_t level);
#endif
    void eventCb(
      const image_view2::MouseEvent::ConstPtr& event_msg);
    void pointArrayToMask(std::vector<cv::Point2d>& points,
                          cv::Mat& mask);
    // `PubT` is `ros::Publisher` (ROS1, type-erased, called as
    // `pub.publish(...)`) or `rclcpp::Publisher<MsgT>::SharedPtr` (ROS2,
    // called as `pub->publish(...)`) -- see ROS1_ROS2_COMPAT::publishMsg() below.
    template<typename PubT>
    void publishMonoImage(PubT& pub,
                          cv::Mat& image,
                          const std_msgs::Header& header);
    template<typename PubT>
    void publishRectFromMaskImage(PubT& pub,
                                  cv::Mat& image,
                                  const std_msgs::Header& header);
    ////////////////////////////////////////////////////////
    // drawing helper methods
    ////////////////////////////////////////////////////////
    void drawLineStrip(const image_view2::ImageMarker2::ConstPtr& marker,
                       std::vector<CvScalar>& colors,
                       std::vector<CvScalar>::iterator& col_it);
    void drawLineList(const image_view2::ImageMarker2::ConstPtr& marker,
                      std::vector<CvScalar>& colors,
                      std::vector<CvScalar>::iterator& col_it);
    void drawPolygon(const image_view2::ImageMarker2::ConstPtr& marker,
                     std::vector<CvScalar>& colors,
                     std::vector<CvScalar>::iterator& col_it);
    void drawPoints(const image_view2::ImageMarker2::ConstPtr& marker,
                    std::vector<CvScalar>& colors,
                    std::vector<CvScalar>::iterator& col_it);
    void drawFrames(const image_view2::ImageMarker2::ConstPtr& marker,
                    std::vector<CvScalar>& colors,
                    std::vector<CvScalar>::iterator& col_it);
    void drawText(const image_view2::ImageMarker2::ConstPtr& marker,
                    std::vector<CvScalar>& colors,
                    std::vector<CvScalar>::iterator& col_it);
    void drawLineStrip3D(const image_view2::ImageMarker2::ConstPtr& marker,
                         std::vector<CvScalar>& colors,
                         std::vector<CvScalar>::iterator& col_it);
    void drawLineList3D(const image_view2::ImageMarker2::ConstPtr& marker,
                        std::vector<CvScalar>& colors,
                        std::vector<CvScalar>::iterator& col_it);
    void drawPolygon3D(const image_view2::ImageMarker2::ConstPtr& marker,
                       std::vector<CvScalar>& colors,
                       std::vector<CvScalar>::iterator& col_it);
    void drawPoints3D(const image_view2::ImageMarker2::ConstPtr& marker,
                       std::vector<CvScalar>& colors,
                       std::vector<CvScalar>::iterator& col_it);
    void drawText3D(const image_view2::ImageMarker2::ConstPtr& marker,
                    std::vector<CvScalar>& colors,
                    std::vector<CvScalar>::iterator& col_it);
    void drawCircle3D(const image_view2::ImageMarker2::ConstPtr& marker,
                      std::vector<CvScalar>& colors,
                      std::vector<CvScalar>::iterator& col_it);
    void drawCircle(const image_view2::ImageMarker2::ConstPtr& marker);
    void drawMarkers();
    void drawInteraction();
    void drawGrid();
    void cropROI();
    void drawInfo(ROS1_ROS2_COMPAT::RosTime& before_rendering);
    void resolveLocalMarkerQueue();
    bool lookupTransformation(
      std::string frame_id, ROS1_ROS2_COMPAT::RosTime& acquisition_time,
      std::map<std::string, int>& tf_fail,
      geometry_msgs::TransformStamped &transform);
    void processMouseEvent(int event, int x, int y, int flags, void* param);
    void processLeftButtonDown(int x, int y);
    void processMove(int x, int y);
    void processLeftButtonUp(int x, int y);
    void publishMouseInteractionResult();
    void checkMousePos(int& x, int& y);
    void createDistortGridImage();
    V_ImageMarkerMessage local_queue_;
    image_transport::Subscriber image_sub_;
#if ROS_VERSION_MAJOR != 1
    rclcpp::Node::SharedPtr node_;
#endif
    ROS1_ROS2_COMPAT::RosSubscriber<image_view2::MouseEvent> event_sub_;
    ROS1_ROS2_COMPAT::RosSubscriber<sensor_msgs::CameraInfo> info_sub_;
    ROS1_ROS2_COMPAT::RosSubscriber<image_view2::ImageMarker2> marker_sub_;
    std::string marker_topic_;
    boost::circular_buffer<double> times_;
    image_transport::Publisher image_pub_;
    image_transport::Publisher local_image_pub_;
#if ROS_VERSION_MAJOR != 1
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_handle_;
#else
    boost::shared_ptr <dynamic_reconfigure::Server<Config> > srv_;
#endif

    V_ImageMarkerMessage marker_queue_;
    boost::mutex queue_mutex_;
    boost::mutex point_array_mutex_;
    sensor_msgs::ImageConstPtr last_msg_;
    sensor_msgs::CameraInfoConstPtr info_msg_;
    cv_bridge::CvImage img_bridge_;
    boost::mutex image_mutex_;
    int skip_draw_rate_;
    cv::Mat original_image_, image_, draw_;
    cv::Mat distort_grid_mask_;
    int div_u_, div_v_;
    int space_,prev_space_;
    int grid_red_, grid_green_, grid_blue_, prev_red_, prev_green_, prev_blue_;
    int grid_thickness_, prev_thickness_;
    bool fisheye_mode_;

    // shared_ptr (constructed in the ctor body, not the initializer
    // list) because tf2_ros::Buffer's constructor signature differs
    // between ROS1 (just an optional cache-time Duration) and ROS2
    // (needs an rclcpp::Clock::SharedPtr).
    boost::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    boost::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    image_geometry::PinholeCameraModel cam_model_;
    std::vector<std::string> frame_ids_;
    std::vector<cv::Point2d> point_array_;
    boost::mutex info_mutex_;

    // for grabcut selection
    bool selecting_fg_;
    std::vector<cv::Point2d> point_bg_array_;
    std::vector<cv::Point2d> point_fg_array_;
    std::vector<cv::Point2d> poly_points_;
    cv::Point poly_selecting_point_;
    bool poly_selecting_done_;
    cv::Rect rect_bg_;
    cv::Rect rect_fg_;
    std::string window_name_;
    boost::format filename_format_;
    int font_;

    double resize_x_, resize_y_;
    CvRect window_selection_;
    cv::Point2f button_up_pos_;
    int count_;
    bool draw_grid_;
    bool blurry_mode_;
    bool show_info_;
    double tf_timeout_;
    bool region_continuous_publish_;
    bool continuous_ready_;
    bool left_button_clicked_;
    ROS1_ROS2_COMPAT::RosPublisher<geometry_msgs::PointStamped> point_pub_;
    ROS1_ROS2_COMPAT::RosPublisher<sensor_msgs::PointCloud2> point_array_pub_;
    ROS1_ROS2_COMPAT::RosPublisher<geometry_msgs::PolygonStamped> rectangle_pub_;
    ROS1_ROS2_COMPAT::RosPublisher<sensor_msgs::Image> rectangle_img_pub_;
    ROS1_ROS2_COMPAT::RosPublisher<geometry_msgs::PointStamped> move_point_pub_;
    ROS1_ROS2_COMPAT::RosPublisher<sensor_msgs::Image> foreground_mask_pub_;
    ROS1_ROS2_COMPAT::RosPublisher<sensor_msgs::Image> background_mask_pub_;
    ROS1_ROS2_COMPAT::RosPublisher<geometry_msgs::PolygonStamped> foreground_rect_pub_;
    ROS1_ROS2_COMPAT::RosPublisher<geometry_msgs::PolygonStamped> background_rect_pub_;
    ROS1_ROS2_COMPAT::RosPublisher<geometry_msgs::PolygonStamped> line_pub_;
    ROS1_ROS2_COMPAT::RosPublisher<geometry_msgs::PolygonStamped> poly_pub_;
    KEY_MODE mode_;
    bool autosize_;
    bool window_initialized_;

    // for line mode interaction
    boost::mutex poly_point_mutex_;
    boost::mutex line_point_mutex_;
    bool line_select_start_point_;
    bool line_selected_;
    cv::Point line_start_point_;
    cv::Point line_end_point_;
    // thread safe setter
    void updateLineStartPoint(cv::Point p);
    void updateLineEndPoint(cv::Point p);
    cv::Point getLineStartPoint();
    cv::Point getLineEndPoint();
    void publishLinePoints();
    void updateLinePoint(cv::Point p);
    void updatePolyPoint(cv::Point p);
    void updatePolySelectingPoint(cv::Point p);
    void clearPolyPoints();
    void publishPolyPoints();
    void finishSelectingPoly();
    bool isPolySelectingFirstTime();
    bool isSelectingLineStartPoint();
    void resetInteraction();
    ROS1_ROS2_COMPAT::RosServiceServer<std_srvs::Empty> rectangle_mode_srv_;
    ROS1_ROS2_COMPAT::RosServiceServer<std_srvs::Empty> series_mode_srv_;
    ROS1_ROS2_COMPAT::RosServiceServer<std_srvs::Empty> grabcut_mode_srv_;
    ROS1_ROS2_COMPAT::RosServiceServer<std_srvs::Empty> grabcut_rect_mode_srv_;
    ROS1_ROS2_COMPAT::RosServiceServer<std_srvs::Empty> line_mode_srv_;
    ROS1_ROS2_COMPAT::RosServiceServer<std_srvs::Empty> none_mode_srv_;
    ROS1_ROS2_COMPAT::RosServiceServer<std_srvs::Empty> poly_mode_srv_;
    ROS1_ROS2_COMPAT::RosServiceServer<image_view2::ChangeMode> change_mode_srv_;
    bool changeModeServiceCallback(
      image_view2::ChangeModeRequest& req,
      image_view2::ChangeModeResponse& res);
    bool rectangleModeServiceCallback(
      std_srvs::EmptyRequest& req,
      std_srvs::EmptyResponse& res);
    bool seriesModeServiceCallback(
      std_srvs::EmptyRequest& req,
      std_srvs::EmptyResponse& res);
    bool grabcutModeServiceCallback(
      std_srvs::EmptyRequest& req,
      std_srvs::EmptyResponse& res);
    bool grabcutRectModeServiceCallback(
      std_srvs::EmptyRequest& req,
      std_srvs::EmptyResponse& res);
    bool lineModeServiceCallback(
      std_srvs::EmptyRequest& req,
      std_srvs::EmptyResponse& res);
    bool polyModeServiceCallback(
      std_srvs::EmptyRequest& req,
      std_srvs::EmptyResponse& res);
    bool noneModeServiceCallback(
      std_srvs::EmptyRequest& req,
      std_srvs::EmptyResponse& res);
    cv::Point ratioPoint(double x, double y);
    KEY_MODE stringToMode(const std::string& str);
  };

  // Template method definitions (must be visible at the two call sites
  // in image_view2.cpp, which each instantiate a different PubT):
  // foreground_mask_pub_/background_mask_pub_ are
  // ROS1_ROS2_COMPAT::RosPublisher<sensor_msgs::Image>, foreground_rect_pub_/
  // background_rect_pub_ are ROS1_ROS2_COMPAT::RosPublisher<geometry_msgs::PolygonStamped>.
  template<typename PubT>
  void ImageView2::publishMonoImage(PubT& pub,
                                    cv::Mat& image,
                                    const std_msgs::Header& header)
  {
    cv_bridge::CvImage image_bridge(
      header, sensor_msgs::image_encodings::MONO8, image);
    ROS1_ROS2_COMPAT::publishMsg(pub, *image_bridge.toImageMsg());
  }

  template<typename PubT>
  void ImageView2::publishRectFromMaskImage(
    PubT& pub,
    cv::Mat& image,
    const std_msgs::Header& header)
  {
    int min_x = image.cols;
    int min_y = image.rows;
    int max_x = 0;
    int max_y = 0;
    for (int j = 0; j < image.rows; j++) {
      for (int i = 0; i < image.cols; i++) {
        if (image.at<uchar>(j, i) != 0) {
          min_x = std::min(min_x, i);
          min_y = std::min(min_y, j);
          max_x = std::max(max_x, i);
          max_y = std::max(max_y, j);
        }
      }
    }
    geometry_msgs::PolygonStamped poly;
    poly.header = header;
    geometry_msgs::Point32 min_pt, max_pt;
    min_pt.x = min_x;
    min_pt.y = min_y;
    max_pt.x = max_x;
    max_pt.y = max_y;
    poly.polygon.points.push_back(min_pt);
    poly.polygon.points.push_back(max_pt);
    ROS1_ROS2_COMPAT::publishMsg(pub, poly);
  }
}

#endif
