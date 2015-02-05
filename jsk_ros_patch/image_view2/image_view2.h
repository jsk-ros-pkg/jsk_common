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
 *     disclaimer in the documentation and/o2r other materials provided
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

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>

#include <image_view2/ImageMarker2.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <std_msgs/Empty.h>
#include <image_view2/ChangeMode.h>
#include <boost/thread.hpp>
#include <boost/format.hpp>
#include <boost/foreach.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/lambda/lambda.hpp>
#include <pcl/point_types.h>
#include <pcl_ros/publisher.h>

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
    enum KEY_MODE {
      MODE_RECTANGLE,
      MODE_SERIES,
      MODE_SELECT_FORE_AND_BACK,
      MODE_SELECT_FORE_AND_BACK_RECT,
      MODE_LINE
    };
      
    ImageView2();
    ImageView2(ros::NodeHandle& nh);
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
    void pointArrayToMask(std::vector<cv::Point2d>& points,
                          cv::Mat& mask);
    void publishMonoImage(ros::Publisher& pub,
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
    void drawInfo(ros::Time& before_rendering);
    void resolveLocalMarkerQueue();
    bool lookupTransformation(
      std::string frame_id, ros::Time& acquisition_time,
      std::map<std::string, int>& tf_fail,
      tf::StampedTransform &transform);
    void processMouseEvent(int event, int x, int y, int flags, void* param);
    void processLeftButtonDown(int x, int y);
    void processMove(int x, int y);
    void processLeftButtonUp(int x, int y);
    void publishMouseInteractionResult();
    void checkMousePos(int& x, int& y);
    V_ImageMarkerMessage local_queue_;
    image_transport::Subscriber image_sub_;
    ros::Subscriber info_sub_;
    ros::Subscriber marker_sub_;
    std::string marker_topic_;
    boost::circular_buffer<double> times_;
    image_transport::Publisher image_pub_;

    V_ImageMarkerMessage marker_queue_;
    boost::mutex queue_mutex_;
    boost::mutex point_array_mutex_;
    sensor_msgs::ImageConstPtr last_msg_;
    sensor_msgs::CameraInfoConstPtr info_msg_;
    cv_bridge::CvImage img_bridge_;
    boost::mutex image_mutex_;
    int skip_draw_rate_;
    cv::Mat original_image_, image_, draw_;

    tf::TransformListener tf_listener_;
    image_geometry::PinholeCameraModel cam_model_;
    std::vector<std::string> frame_ids_;
    std::vector<cv::Point2d> point_array_;
    boost::mutex info_mutex_;

    // for grabcut selection
    bool selecting_fg_;
    std::vector<cv::Point2d> point_bg_array_;
    std::vector<cv::Point2d> point_fg_array_;
    cv::Rect rect_bg_;
    cv::Rect rect_fg_;
    std::string window_name_;
    boost::format filename_format_;
    int font_;

    double resize_x_, resize_y_;
    CvRect window_selection_;
    cv::Point2f button_up_pos_;
    int count_;
    bool blurry_mode_;
    bool show_info_;
    double tf_timeout_;
    bool region_continuous_publish_;
    bool continuous_ready_;
    bool left_button_clicked_;
    ros::Publisher point_pub_;
    ros::Publisher point_array_pub_;
    ros::Publisher rectangle_pub_;
    ros::Publisher move_point_pub_;
    ros::Publisher foreground_mask_pub_;
    ros::Publisher background_mask_pub_;
    ros::Publisher line_pub_;
    KEY_MODE mode_;
    bool autosize_;
    bool window_initialized_;

    // for line mode interaction
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
    bool isSelectingLineStartPoint();
    void resetInteraction();
    ros::ServiceServer change_mode_srv_;
    bool changeModeServiceCallback(
      image_view2::ChangeModeRequest& req,
      image_view2::ChangeModeResponse& res);
    KEY_MODE stringToMode(const std::string& str);
  };
}

#endif
