// -*- tab-width: 8; indent-tabs-mode: nil; -*-
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

#include "image_view2.h"

namespace image_view2{
  ImageView2::ImageView2() : marker_topic_("image_marker"), filename_format_(""), count_(0), mode_(MODE_RECTANGLE), times_(100), window_initialized_(false)
  {
  }
  
  ImageView2::ImageView2(ros::NodeHandle& nh)
    : marker_topic_("image_marker"), filename_format_(""), count_(0), mode_(MODE_RECTANGLE), times_(100), selecting_fg_(true)
  {
    std::string camera = nh.resolveName("image");
    std::string camera_info = nh.resolveName("camera_info");
    ros::NodeHandle local_nh("~");
    std::string format_string;
    std::string transport;
    image_transport::ImageTransport it(nh);

    point_pub_ = nh.advertise<geometry_msgs::PointStamped>(camera + "/screenpoint",100);
    point_array_pub_ = nh.advertise<sensor_msgs::PointCloud2>(camera + "/screenpoint_array",100);
    rectangle_pub_ = nh.advertise<geometry_msgs::PolygonStamped>(camera + "/screenrectangle",100);
    move_point_pub_ = nh.advertise<geometry_msgs::PointStamped>(camera + "/movepoint", 100);
    foreground_mask_pub_ = nh.advertise<sensor_msgs::Image>(camera + "/foreground", 100);
    background_mask_pub_ = nh.advertise<sensor_msgs::Image>(camera + "/background", 100);
    local_nh.param("window_name", window_name_, std::string("image_view2 [")+camera+std::string("]"));
    local_nh.param("skip_draw_rate", skip_draw_rate_, 0);
    local_nh.param("autosize", autosize_, false);
    local_nh.param("image_transport", transport, std::string("raw"));
    local_nh.param("blurry", blurry_mode_, false);

    local_nh.param("filename_format", format_string, std::string("frame%04i.jpg"));
    local_nh.param("use_window", use_window, true);
    local_nh.param("show_info", show_info_, false);

    double xx,yy;
    local_nh.param("resize_scale_x", xx, 1.0);
    local_nh.param("resize_scale_y", yy, 1.0);
    local_nh.param("tf_timeout", tf_timeout_, 1.0);
    
    std::string interaction_mode;
    local_nh.param("interaction_mode", interaction_mode, std::string("rectangle"));
    if (interaction_mode == "rectangle") {
      setMode(image_view2::ImageView2::MODE_RECTANGLE);
    }
    else if (interaction_mode == "freeform" ||
             interaction_mode == "series") {
      setMode(image_view2::ImageView2::MODE_SERIES);
    }
    else if (interaction_mode == "grabcut") {
      setMode(image_view2::ImageView2::MODE_SELECT_FORE_AND_BACK);
    }
    else if (interaction_mode == "grabcut_rect") {
      setMode(image_view2::ImageView2::MODE_SELECT_FORE_AND_BACK_RECT);
    }
    
    resize_x_ = 1.0/xx;
    resize_y_ = 1.0/yy;
    filename_format_.parse(format_string);

    if ( use_window ) {
      font_ = cv::FONT_HERSHEY_DUPLEX;
      window_selection_.x = window_selection_.y =
        window_selection_.height = window_selection_.width = 0;
    }

    image_sub_ = it.subscribe(camera, 1, &ImageView2::imageCb, this, transport);
    info_sub_ = nh.subscribe(camera_info, 1, &ImageView2::infoCb, this);
    marker_sub_ = nh.subscribe(marker_topic_, 10, &ImageView2::markerCb, this);

    image_pub_ = it.advertise("image_marked", 1);
  }

  ImageView2::~ImageView2()
  {
    if ( use_window ) {
      cv::destroyWindow(window_name_.c_str());
    }
  }

  void ImageView2::markerCb(const image_view2::ImageMarker2ConstPtr& marker)
  {
    ROS_DEBUG("markerCb");
    // convert lifetime to duration from Time(0)
    if(marker->lifetime != ros::Duration(0))
      boost::const_pointer_cast<image_view2::ImageMarker2>(marker)->lifetime = (ros::Time::now() - ros::Time(0)) + marker->lifetime;
    {
      boost::mutex::scoped_lock lock(queue_mutex_);
      marker_queue_.push_back(marker);
    }
    redraw();
  }

  void ImageView2::infoCb(const sensor_msgs::CameraInfoConstPtr& msg) {
    ROS_DEBUG("infoCb");
    boost::mutex::scoped_lock lock(info_mutex_);
    info_msg_ = msg;
  }

  bool ImageView2::lookupTransformation(
    std::string frame_id, ros::Time& acquisition_time,
    std::map<std::string, int>& tf_fail,
    tf::StampedTransform &transform)
  {
    ros::Duration timeout(tf_timeout_); // wait 0.5 sec
    try {
      ros::Time tm;
      tf_listener_.getLatestCommonTime(cam_model_.tfFrame(), frame_id, tm, NULL);
      tf_listener_.waitForTransform(cam_model_.tfFrame(), frame_id,
                                    acquisition_time, timeout);
      tf_listener_.lookupTransform(cam_model_.tfFrame(), frame_id,
                                   acquisition_time, transform);
      tf_fail[frame_id]=0;
      return true;
    }
    catch (tf::TransformException& ex) {
      tf_fail[frame_id]++;
      if ( tf_fail[frame_id] < 5 ) {
        ROS_ERROR("[image_view2] TF exception:\n%s", ex.what());
      } else {
        ROS_DEBUG("[image_view2] TF exception:\n%s", ex.what());
      }
      return false;
    }
  }
  
  void ImageView2::drawCircle(const image_view2::ImageMarker2::ConstPtr& marker)
  {
    cv::Point2d uv = cv::Point2d(marker->position.x, marker->position.y);
    if ( blurry_mode_ ) {
      int s0 = (marker->width == 0 ? DEFAULT_LINE_WIDTH : marker->width);
      CvScalar co = MsgToRGB(marker->outline_color);
      for (int s1 = s0*10; s1 >= s0; s1--) {
        double m = pow((1.0-((double)(s1 - s0))/(s0*9)),2);
        cv::circle(draw_, uv,
                   (marker->scale == 0 ? DEFAULT_CIRCLE_SCALE : marker->scale),
                   CV_RGB(co.val[2] * m,co.val[1] * m,co.val[0] * m),
                   s1);
      }
    } else {
      cv::circle(draw_, uv,
                 (marker->scale == 0 ? DEFAULT_CIRCLE_SCALE : marker->scale),
                 MsgToRGB(marker->outline_color),
                 (marker->width == 0 ? DEFAULT_LINE_WIDTH : marker->width));
      if (marker->filled) {
        cv::circle(draw_, uv,
                   (marker->scale == 0 ? DEFAULT_CIRCLE_SCALE : marker->scale) - (marker->width == 0 ? DEFAULT_LINE_WIDTH : marker->width) / 2.0,
                   MsgToRGB(marker->fill_color),
                   -1);
      }
    }
  }

  void ImageView2::drawLineStrip(const image_view2::ImageMarker2::ConstPtr& marker,
                                 std::vector<CvScalar>& colors,
                                 std::vector<CvScalar>::iterator& col_it)
  {
    cv::Point2d p0, p1;
    if ( blurry_mode_ ) {
      int s0 = (marker->width == 0 ? DEFAULT_LINE_WIDTH : marker->width);
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
        cv::line(draw_, p0, p1, *col_it, (marker->width == 0 ? DEFAULT_LINE_WIDTH : marker->width));
        p0 = p1;
        if(++col_it == colors.end()) col_it = colors.begin();
      }
    }
  }

  void ImageView2::drawLineList(const image_view2::ImageMarker2::ConstPtr& marker,
                                std::vector<CvScalar>& colors,
                                std::vector<CvScalar>::iterator& col_it)
  {
    cv::Point2d p0, p1;
    if ( blurry_mode_ ) {
      int s0 = (marker->width == 0 ? DEFAULT_LINE_WIDTH : marker->width);
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
        cv::line(draw_, p0, p1, *col_it, (marker->width == 0 ? DEFAULT_LINE_WIDTH : marker->width));
        it++;
        if(++col_it == colors.end()) col_it = colors.begin();
      }
    }
  }

  void ImageView2::drawPolygon(const image_view2::ImageMarker2::ConstPtr& marker,
                               std::vector<CvScalar>& colors,
                               std::vector<CvScalar>::iterator& col_it)
  {
    cv::Point2d p0, p1;
    if ( blurry_mode_ ) {
      int s0 = (marker->width == 0 ? DEFAULT_LINE_WIDTH : marker->width);
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
      std::vector<cv::Point> points;

      if (marker->filled) {
        points.push_back(cv::Point(it->x, it->y));
      }
      p0 = cv::Point2d(it->x, it->y); it++;
      for ( ; it!= end; it++ ) {
        p1 = cv::Point2d(it->x, it->y);
        if (marker->filled) {
          points.push_back(cv::Point(it->x, it->y));
        }
        cv::line(draw_, p0, p1, *col_it, (marker->width == 0 ? DEFAULT_LINE_WIDTH : marker->width));
        p0 = p1;
        if(++col_it == colors.end()) col_it = colors.begin();
      }
      it = marker->points.begin();
      p1 = cv::Point2d(it->x, it->y);
      cv::line(draw_, p0, p1, *col_it, (marker->width == 0 ? DEFAULT_LINE_WIDTH : marker->width));
      if (marker->filled) {
        cv::fillConvexPoly(draw_, points.data(), points.size(), MsgToRGB(marker->fill_color));
      }
    }
  }

  void ImageView2::drawPoints(const image_view2::ImageMarker2::ConstPtr& marker,
                              std::vector<CvScalar>& colors,
                              std::vector<CvScalar>::iterator& col_it)
  {
    BOOST_FOREACH(geometry_msgs::Point p, marker->points) {
      cv::Point2d uv = cv::Point2d(p.x, p.y);
      if ( blurry_mode_ ) {
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
  }

  void ImageView2::drawFrames(const image_view2::ImageMarker2::ConstPtr& marker,
                              std::vector<CvScalar>& colors,
                              std::vector<CvScalar>::iterator& col_it)
  {
    static std::map<std::string, int> tf_fail;
    BOOST_FOREACH(std::string frame_id, marker->frames) {
      tf::StampedTransform transform;
      ros::Time acquisition_time = last_msg_->header.stamp;
      if(!lookupTransformation(frame_id, acquisition_time, tf_fail, transform)) {
        return;
      }
      // center point
      tf::Point pt = transform.getOrigin();
      cv::Point3d pt_cv(pt.x(), pt.y(), pt.z());
      cv::Point2d uv;
      uv = cam_model_.project3dToPixel(pt_cv);

      static const int RADIUS = 3;
      cv::circle(draw_, uv, RADIUS, DEFAULT_COLOR, -1);

      // x, y, z
      cv::Point2d uv0, uv1, uv2;
      tf::Stamped<tf::Point> pin, pout;

      // x
      pin = tf::Stamped<tf::Point>(tf::Point(0.05, 0, 0), acquisition_time, frame_id);
      tf_listener_.transformPoint(cam_model_.tfFrame(), pin, pout);
      uv0 = cam_model_.project3dToPixel(cv::Point3d(pout.x(), pout.y(), pout.z()));
      // y
      pin = tf::Stamped<tf::Point>(tf::Point(0, 0.05, 0), acquisition_time, frame_id);
      tf_listener_.transformPoint(cam_model_.tfFrame(), pin, pout);
      uv1 = cam_model_.project3dToPixel(cv::Point3d(pout.x(), pout.y(), pout.z()));

      // z
      pin = tf::Stamped<tf::Point>(tf::Point(0, 0, 0.05), acquisition_time, frame_id);
      tf_listener_.transformPoint(cam_model_.tfFrame(), pin, pout);
      uv2 = cam_model_.project3dToPixel(cv::Point3d(pout.x(), pout.y(), pout.z()));

      // draw
      if ( blurry_mode_ ) {
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
  }
  
  void ImageView2::drawText(const image_view2::ImageMarker2::ConstPtr& marker,
                            std::vector<CvScalar>& colors,
                            std::vector<CvScalar>::iterator& col_it)
  {
    cv::Size text_size;
    int baseline;
    float scale = marker->scale;
    if ( scale == 0 ) scale = 1.0;
    text_size = cv::getTextSize(marker->text.c_str(), font_,
                                scale, scale, &baseline);
    cv::Point origin = cv::Point(marker->position.x - text_size.width/2,
                                 marker->position.y - baseline-3);
    cv::putText(draw_, marker->text.c_str(), origin, font_, scale, DEFAULT_COLOR);
  }

  void ImageView2::drawLineStrip3D(const image_view2::ImageMarker2::ConstPtr& marker,
                                   std::vector<CvScalar>& colors,
                                   std::vector<CvScalar>::iterator& col_it)
  {
    static std::map<std::string, int> tf_fail;
    std::string frame_id = marker->points3D.header.frame_id;
    tf::StampedTransform transform;
    ros::Time acquisition_time = last_msg_->header.stamp;
    //ros::Time acquisition_time = msg->points3D.header.stamp;
    ros::Duration timeout(tf_timeout_); // wait 0.5 sec
    try {
      ros::Time tm;
      tf_listener_.getLatestCommonTime(cam_model_.tfFrame(), frame_id, tm, NULL);
      ros::Duration diff = ros::Time::now() - tm;
      if ( diff > ros::Duration(1.0) ) { return; }
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
      return;
    }
    std::vector<geometry_msgs::Point> points2D;
    BOOST_FOREACH(geometry_msgs::Point p, marker->points3D.points) {
      tf::Point pt = transform.getOrigin();
      geometry_msgs::PointStamped pt_cam, pt_;
      pt_cam.header.frame_id = cam_model_.tfFrame();
      pt_cam.header.stamp = acquisition_time;
      pt_cam.point.x = pt.x();
      pt_cam.point.y = pt.y();
      pt_cam.point.z = pt.z();
      tf_listener_.transformPoint(frame_id, pt_cam, pt_);

      cv::Point2d uv;
      tf::Stamped<tf::Point> pin, pout;
      pin = tf::Stamped<tf::Point>(tf::Point(pt_.point.x + p.x, pt_.point.y + p.y, pt_.point.z + p.z), acquisition_time, frame_id);
      tf_listener_.transformPoint(cam_model_.tfFrame(), pin, pout);
      uv = cam_model_.project3dToPixel(cv::Point3d(pout.x(), pout.y(), pout.z()));
      geometry_msgs::Point point2D;
      point2D.x = uv.x;
      point2D.y = uv.y;
      points2D.push_back(point2D);
    }
    cv::Point2d p0, p1;
    std::vector<geometry_msgs::Point>::const_iterator it = points2D.begin();
    std::vector<geometry_msgs::Point>::const_iterator end = points2D.end();
    p0 = cv::Point2d(it->x, it->y); it++;
    for ( ; it!= end; it++ ) {
      p1 = cv::Point2d(it->x, it->y);
      cv::line(draw_, p0, p1, *col_it, (marker->width == 0 ? DEFAULT_LINE_WIDTH : marker->width));
      p0 = p1;
      if(++col_it == colors.end()) col_it = colors.begin();
    }
  }

  void ImageView2::drawLineList3D(const image_view2::ImageMarker2::ConstPtr& marker,
                                  std::vector<CvScalar>& colors,
                                  std::vector<CvScalar>::iterator& col_it)
  {
    static std::map<std::string, int> tf_fail;
    std::string frame_id = marker->points3D.header.frame_id;
    tf::StampedTransform transform;
    ros::Time acquisition_time = last_msg_->header.stamp;
    if(!lookupTransformation(frame_id, acquisition_time, tf_fail, transform)) {
      return;
    }
    std::vector<geometry_msgs::Point> points2D;
    BOOST_FOREACH(geometry_msgs::Point p, marker->points3D.points) {
      tf::Point pt = transform.getOrigin();
      geometry_msgs::PointStamped pt_cam, pt_;
      pt_cam.header.frame_id = cam_model_.tfFrame();
      pt_cam.header.stamp = acquisition_time;
      pt_cam.point.x = pt.x();
      pt_cam.point.y = pt.y();
      pt_cam.point.z = pt.z();
      tf_listener_.transformPoint(frame_id, pt_cam, pt_);

      cv::Point2d uv;
      tf::Stamped<tf::Point> pin, pout;
      pin = tf::Stamped<tf::Point>(tf::Point(pt_.point.x + p.x, pt_.point.y + p.y, pt_.point.z + p.z), acquisition_time, frame_id);
      tf_listener_.transformPoint(cam_model_.tfFrame(), pin, pout);
      uv = cam_model_.project3dToPixel(cv::Point3d(pout.x(), pout.y(), pout.z()));
      geometry_msgs::Point point2D;
      point2D.x = uv.x;
      point2D.y = uv.y;
      points2D.push_back(point2D);
    }
    cv::Point2d p0, p1;
    std::vector<geometry_msgs::Point>::const_iterator it = points2D.begin();
    std::vector<geometry_msgs::Point>::const_iterator end = points2D.end();
    for ( ; it!= end; ) {
      p0 = cv::Point2d(it->x, it->y); it++;
      if ( it != end ) p1 = cv::Point2d(it->x, it->y);
      cv::line(draw_, p0, p1, *col_it, (marker->width == 0 ? DEFAULT_LINE_WIDTH : marker->width));
      it++;
      if(++col_it == colors.end()) col_it = colors.begin();
    }
  }
  
  void ImageView2::drawPolygon3D(const image_view2::ImageMarker2::ConstPtr& marker,
                                 std::vector<CvScalar>& colors,
                                 std::vector<CvScalar>::iterator& col_it)
  {
    static std::map<std::string, int> tf_fail;
    std::string frame_id = marker->points3D.header.frame_id;
    tf::StampedTransform transform;
    ros::Time acquisition_time = last_msg_->header.stamp;
    if(!lookupTransformation(frame_id, acquisition_time, tf_fail, transform)) {
      return;
    }
    std::vector<geometry_msgs::Point> points2D;
    BOOST_FOREACH(geometry_msgs::Point p, marker->points3D.points) {
      tf::Point pt = transform.getOrigin();
      geometry_msgs::PointStamped pt_cam, pt_;
      pt_cam.header.frame_id = cam_model_.tfFrame();
      pt_cam.header.stamp = acquisition_time;
      pt_cam.point.x = pt.x();
      pt_cam.point.y = pt.y();
      pt_cam.point.z = pt.z();
      tf_listener_.transformPoint(frame_id, pt_cam, pt_);

      cv::Point2d uv;
      tf::Stamped<tf::Point> pin, pout;
      pin = tf::Stamped<tf::Point>(tf::Point(pt_.point.x + p.x, pt_.point.y + p.y, pt_.point.z + p.z), acquisition_time, frame_id);
      tf_listener_.transformPoint(cam_model_.tfFrame(), pin, pout);
      uv = cam_model_.project3dToPixel(cv::Point3d(pout.x(), pout.y(), pout.z()));
      geometry_msgs::Point point2D;
      point2D.x = uv.x;
      point2D.y = uv.y;
      points2D.push_back(point2D);
    }
    cv::Point2d p0, p1;
    std::vector<geometry_msgs::Point>::const_iterator it = points2D.begin();
    std::vector<geometry_msgs::Point>::const_iterator end = points2D.end();
    std::vector<cv::Point> points;

    if (marker->filled) {
      points.push_back(cv::Point(it->x, it->y));
    }
    p0 = cv::Point2d(it->x, it->y); it++;
    for ( ; it!= end; it++ ) {
      p1 = cv::Point2d(it->x, it->y);
      if (marker->filled) {
        points.push_back(cv::Point(it->x, it->y));
      }
      cv::line(draw_, p0, p1, *col_it, (marker->width == 0 ? DEFAULT_LINE_WIDTH : marker->width));
      p0 = p1;
      if(++col_it == colors.end()) col_it = colors.begin();
    }
    it = points2D.begin();
    p1 = cv::Point2d(it->x, it->y);
    cv::line(draw_, p0, p1, *col_it, (marker->width == 0 ? DEFAULT_LINE_WIDTH : marker->width));
    if (marker->filled) {
      cv::fillConvexPoly(draw_, points.data(), points.size(), MsgToRGB(marker->fill_color));
    }
  }

  void ImageView2::drawPoints3D(const image_view2::ImageMarker2::ConstPtr& marker,
                                std::vector<CvScalar>& colors,
                                std::vector<CvScalar>::iterator& col_it)
  {
    static std::map<std::string, int> tf_fail;
    std::string frame_id = marker->points3D.header.frame_id;
    tf::StampedTransform transform;
    ros::Time acquisition_time = last_msg_->header.stamp;
    if(!lookupTransformation(frame_id, acquisition_time, tf_fail, transform)) {
      return;
    }
    BOOST_FOREACH(geometry_msgs::Point p, marker->points3D.points) {
      tf::Point pt = transform.getOrigin();
      geometry_msgs::PointStamped pt_cam, pt_;
      pt_cam.header.frame_id = cam_model_.tfFrame();
      pt_cam.header.stamp = acquisition_time;
      pt_cam.point.x = pt.x();
      pt_cam.point.y = pt.y();
      pt_cam.point.z = pt.z();
      tf_listener_.transformPoint(frame_id, pt_cam, pt_);

      cv::Point2d uv;
      tf::Stamped<tf::Point> pin, pout;
      pin = tf::Stamped<tf::Point>(tf::Point(pt_.point.x + p.x, pt_.point.y + p.y, pt_.point.z + p.z), acquisition_time, frame_id);
      tf_listener_.transformPoint(cam_model_.tfFrame(), pin, pout);
      uv = cam_model_.project3dToPixel(cv::Point3d(pout.x(), pout.y(), pout.z()));
      cv::circle(draw_, uv, (marker->scale == 0 ? 3 : marker->scale) , *col_it, -1);
    }
  }

  void ImageView2::drawText3D(const image_view2::ImageMarker2::ConstPtr& marker,
                              std::vector<CvScalar>& colors,
                              std::vector<CvScalar>::iterator& col_it)
  {
    static std::map<std::string, int> tf_fail;
    std::string frame_id = marker->position3D.header.frame_id;
    tf::StampedTransform transform;
    ros::Time acquisition_time = last_msg_->header.stamp;
    if(!lookupTransformation(frame_id, acquisition_time, tf_fail, transform)) {
      return;
    }
    tf::Point pt = transform.getOrigin();
    geometry_msgs::PointStamped pt_cam, pt_;
    pt_cam.header.frame_id = cam_model_.tfFrame();
    pt_cam.header.stamp = acquisition_time;
    pt_cam.point.x = pt.x();
    pt_cam.point.y = pt.y();
    pt_cam.point.z = pt.z();
    tf_listener_.transformPoint(frame_id, pt_cam, pt_);

    cv::Point2d uv;
    tf::Stamped<tf::Point> pin, pout;
    pin = tf::Stamped<tf::Point>(tf::Point(pt_.point.x + marker->position3D.point.x, pt_.point.y + marker->position3D.point.y, pt_.point.z + marker->position3D.point.z), acquisition_time, frame_id);
    tf_listener_.transformPoint(cam_model_.tfFrame(), pin, pout);
    uv = cam_model_.project3dToPixel(cv::Point3d(pout.x(), pout.y(), pout.z()));
    cv::Size text_size;
    int baseline;
    float scale = marker->scale;
    if ( scale == 0 ) scale = 1.0;
    text_size = cv::getTextSize(marker->text.c_str(), font_,
                                scale, scale, &baseline);
    cv::Point origin = cv::Point(uv.x - text_size.width/2,
                                 uv.y - baseline-3);
    cv::putText(draw_, marker->text.c_str(), origin, font_, scale, CV_RGB(0,255,0),3);
  }

  void ImageView2::drawCircle3D(const image_view2::ImageMarker2::ConstPtr& marker,
                                std::vector<CvScalar>& colors,
                                std::vector<CvScalar>::iterator& col_it)
  {
    static std::map<std::string, int> tf_fail;
    std::string frame_id = marker->pose.header.frame_id;
    geometry_msgs::PoseStamped pose;
    ros::Time acquisition_time = last_msg_->header.stamp;
    ros::Duration timeout(tf_timeout_); // wait 0.5 sec
    try {
      ros::Time tm;
      tf_listener_.getLatestCommonTime(cam_model_.tfFrame(), frame_id, tm, NULL);
      tf_listener_.waitForTransform(cam_model_.tfFrame(), frame_id,
                                    acquisition_time, timeout);
      tf_listener_.transformPose(cam_model_.tfFrame(),
                                 acquisition_time, marker->pose, frame_id, pose);
      tf_fail[frame_id]=0;
    }
    catch (tf::TransformException& ex) {
      tf_fail[frame_id]++;
      if ( tf_fail[frame_id] < 5 ) {
        ROS_ERROR("[image_view2] TF exception:\n%s", ex.what());
      } else {
        ROS_DEBUG("[image_view2] TF exception:\n%s", ex.what());
      }
      return;
    }

    tf::Quaternion q;
    tf::quaternionMsgToTF(pose.pose.orientation, q);
    tf::Matrix3x3 rot = tf::Matrix3x3(q);
    double angle = (marker->arc == 0 ? 360.0 :marker->angle);
    double scale = (marker->scale == 0 ? DEFAULT_CIRCLE_SCALE : marker->scale);
    int N = 100;
    std::vector< std::vector<cv::Point2i> > ptss;
    std::vector<cv::Point2i> pts;

    for (int i=0; i<N; ++i) {
      double th = angle * i / N * TFSIMD_RADS_PER_DEG;
      tf::Vector3 v = rot * tf::Vector3(scale * tfCos(th), scale * tfSin(th),0);
      cv::Point2d pt = cam_model_.project3dToPixel(cv::Point3d(pose.pose.position.x + v.getX(), pose.pose.position.y + v.getY(), pose.pose.position.z + v.getZ()));
      pts.push_back(cv::Point2i((int)pt.x, (int)pt.y));
    }
    ptss.push_back(pts);

    cv::polylines(draw_, ptss, (marker->arc == 0 ? true : false), MsgToRGB(marker->outline_color), (marker->width == 0 ? DEFAULT_LINE_WIDTH : marker->width));

    if (marker->filled) {
      if(marker->arc != 0){
        cv::Point2d pt = cam_model_.project3dToPixel(cv::Point3d(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z));
        pts.push_back(cv::Point2i((int)pt.x, (int)pt.y));
        ptss.clear();
        ptss.push_back(pts);
      }
      cv::fillPoly(draw_, ptss, MsgToRGB(marker->fill_color));
    }
  }

  void ImageView2::resolveLocalMarkerQueue()
  {
    {
      boost::mutex::scoped_lock lock(queue_mutex_);

      while ( ! marker_queue_.empty() ) {
        // remove marker by namespace and id
        V_ImageMarkerMessage::iterator new_msg = marker_queue_.begin();
        V_ImageMarkerMessage::iterator message_it = local_queue_.begin();
        for ( ; message_it < local_queue_.end(); ++message_it ) {
          if((*new_msg)->ns == (*message_it)->ns && (*new_msg)->id == (*message_it)->id)
            message_it = local_queue_.erase(message_it);
        }
        local_queue_.push_back(*new_msg);
        marker_queue_.erase(new_msg);
        // if action == REMOVE and id == -1, clear all marker_queue
        if ( (*new_msg)->action == image_view2::ImageMarker2::REMOVE &&
             (*new_msg)->id == -1 ) {
          local_queue_.clear();
        }
      }
    }

    // check lifetime and remove REMOVE-type marker msg
    for(V_ImageMarkerMessage::iterator it = local_queue_.begin(); it < local_queue_.end(); it++) {
      if((*it)->action == image_view2::ImageMarker2::REMOVE ||
         ((*it)->lifetime.toSec() != 0.0 && (*it)->lifetime.toSec() < ros::Time::now().toSec())) {
        it = local_queue_.erase(it);
      }
    }
  }
  
  void ImageView2::drawMarkers()
  {
    resolveLocalMarkerQueue();
    if ( !local_queue_.empty() )
    {
      V_ImageMarkerMessage::iterator message_it = local_queue_.begin();
      V_ImageMarkerMessage::iterator message_end = local_queue_.end();
      ROS_DEBUG("markers = %ld", local_queue_.size());
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

        // check camera_info
        if( marker->type == image_view2::ImageMarker2::FRAMES ||
            marker->type == image_view2::ImageMarker2::LINE_STRIP3D ||
            marker->type == image_view2::ImageMarker2::LINE_LIST3D ||
            marker->type == image_view2::ImageMarker2::POLYGON3D ||
            marker->type == image_view2::ImageMarker2::POINTS3D ||
            marker->type == image_view2::ImageMarker2::TEXT3D ||
            marker->type == image_view2::ImageMarker2::CIRCLE3D) {
          {
            boost::mutex::scoped_lock lock(info_mutex_);
            if (!info_msg_) {
              ROS_WARN("[image_view2] camera_info could not found");
              continue;
            }
            cam_model_.fromCameraInfo(info_msg_);
          }
        }
        // CIRCLE, LINE_STRIP, LINE_LIST, POLYGON, POINTS
        switch ( marker->type ) {
        case image_view2::ImageMarker2::CIRCLE: {
          drawCircle(marker);
          break;
        }
        case image_view2::ImageMarker2::LINE_STRIP: {
          drawLineStrip(marker, colors, col_it);
          break;
        }
        case image_view2::ImageMarker2::LINE_LIST: {
          drawLineList(marker, colors, col_it);
          break;
        }
        case image_view2::ImageMarker2::POLYGON: {
          drawPolygon(marker, colors, col_it);
          break;
        }
        case image_view2::ImageMarker2::POINTS: {
          drawPoints(marker, colors, col_it);
          break;
        }
        case image_view2::ImageMarker2::FRAMES: {
          drawFrames(marker, colors, col_it);
          break;
        }
        case image_view2::ImageMarker2::TEXT: {
          drawText(marker, colors, col_it);
          break;
        }
        case image_view2::ImageMarker2::LINE_STRIP3D: {
          drawLineStrip3D(marker, colors, col_it);
          break;
        }
        case image_view2::ImageMarker2::LINE_LIST3D: {
          drawLineList3D(marker, colors, col_it);
          break;
        }
        case image_view2::ImageMarker2::POLYGON3D: {
          drawPolygon3D(marker, colors, col_it);
          break;
        }
        case image_view2::ImageMarker2::POINTS3D: {
          drawPoints3D(marker, colors, col_it);
          break;
        }
        case image_view2::ImageMarker2::TEXT3D: {
          drawText3D(marker, colors, col_it);
          break;
        }
        case image_view2::ImageMarker2::CIRCLE3D: {
          drawCircle3D(marker, colors, col_it);
          break;
        }
        default: {
          ROS_WARN("Undefined Marker type(%d)", marker->type);
          break;
        }
        }
      }
    }    
  }

  void ImageView2::drawInteraction()
  {
    if (mode_ == MODE_RECTANGLE) {
      cv::rectangle(draw_, cv::Point(window_selection_.x, window_selection_.y),
                    cv::Point(window_selection_.x + window_selection_.width,
                              window_selection_.y + window_selection_.height),
                    USER_ROI_COLOR, 3, 8, 0);
    }
    else if (mode_ == MODE_SERIES) {
      if (point_array_.size() > 1) {
        cv::Point2d from, to;
        from = point_array_[0];
        for (size_t i = 1; i < point_array_.size(); i++) {
          to = point_array_[i];
          cv::line(draw_, from, to, USER_ROI_COLOR, 2, 8, 0);
          from = to;
        }
      }
    }
    else if (mode_ == MODE_SELECT_FORE_AND_BACK) {
      boost::mutex::scoped_lock lock(point_array_mutex_);
      if (point_fg_array_.size() > 1) {
        cv::Point2d from, to;
        from = point_fg_array_[0];
        for (size_t i = 1; i < point_fg_array_.size(); i++) {
          to = point_fg_array_[i];
          cv::line(draw_, from, to, CV_RGB(255, 0, 0), 8, 8, 0);
          from = to;
        }
      }
      if (point_bg_array_.size() > 1) {
        cv::Point2d from, to;
        from = point_bg_array_[0];
        for (size_t i = 1; i < point_bg_array_.size(); i++) {
          to = point_bg_array_[i];
          cv::line(draw_, from, to, CV_RGB(0, 255, 0), 8, 8, 0);
          from = to;
        }
      }
    }
    else if (mode_ == MODE_SELECT_FORE_AND_BACK_RECT) {
      boost::mutex::scoped_lock lock(point_array_mutex_);
      if (rect_fg_.width != 0 && rect_fg_.height != 0) {
        cv::rectangle(draw_, rect_fg_, CV_RGB(255, 0, 0), 4);
      }
      if (rect_bg_.width != 0 && rect_bg_.height != 0) {
        cv::rectangle(draw_, rect_bg_, CV_RGB(0, 255, 0), 4);
      }
    }
  }

  void ImageView2::drawInfo(ros::Time& before_rendering)
  {
    static ros::Time last_time;
    static std::string info_str_1, info_str_2;
    // update info_str_1, info_str_2 if possible
    if ( show_info_ && times_.size() > 0 && ( before_rendering.toSec() - last_time.toSec() > 2 ) ) {
      int n = times_.size();
      double mean = 0, rate = 1.0, std_dev = 0.0, max_delta, min_delta;

      std::for_each( times_.begin(), times_.end(), (mean += boost::lambda::_1) );
      mean /= n;
      rate /= mean;

      std::for_each( times_.begin(), times_.end(), (std_dev += (boost::lambda::_1 - mean)*(boost::lambda::_1 - mean) ) );
      std_dev = sqrt(std_dev/n);
      min_delta = *std::min_element(times_.begin(), times_.end());
      max_delta = *std::max_element(times_.begin(), times_.end());

      std::stringstream f1, f2;
      f1.precision(3); f1 << std::fixed;
      f2.precision(3); f2 << std::fixed;
      f1 << "" << image_sub_.getTopic() << " : rate:" << rate;
      f2 << "min:" << min_delta << "s max: " << max_delta << "s std_dev: " << std_dev << "s n: " << n;
      info_str_1 = f1.str();
      info_str_2 = f2.str();
      ROS_INFO_STREAM(info_str_1 + " " + info_str_2);
      times_.clear();
      last_time = before_rendering;
    }
    if (!info_str_1.empty() && !info_str_2.empty()) {
      cv::putText(image_, info_str_1.c_str(), cv::Point(10,image_.rows-34), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar::all(255), 2);
      cv::putText(image_, info_str_2.c_str(), cv::Point(10,image_.rows-10), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar::all(255), 2);
    }
  }
  
  void ImageView2::redraw()
  {
    if (original_image_.empty()) {
      ROS_WARN("no image is available yet");
      return;
    }
    ros::Time before_rendering = ros::Time::now();
    original_image_.copyTo(image_);
    // Draw Section
    if ( blurry_mode_ ) {
      draw_ = cv::Mat(image_.size(), image_.type(), CV_RGB(0,0,0));
    } else {
      draw_ = image_;
    }
    drawMarkers();
    drawInteraction();

    if ( blurry_mode_ ) cv::addWeighted(image_, 0.9, draw_, 1.0, 0.0, image_);
    if ( use_window ) {
      if (show_info_) {
        drawInfo(before_rendering);
      }
    }
    cv_bridge::CvImage out_msg;
    out_msg.header   = last_msg_->header;
    out_msg.encoding = "bgr8";
    out_msg.image    = image_;
    image_pub_.publish(out_msg.toImageMsg());
  }
  
  void ImageView2::imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    ROS_DEBUG("imageCb");
    static int count = 0;
    if (count < skip_draw_rate_) {
      count++;
      return;
    }
    else {
      count = 0;
    }
    static ros::Time old_time;
    times_.push_front(ros::Time::now().toSec() - old_time.toSec());
    old_time = ros::Time::now();

    if(old_time.toSec() - ros::Time::now().toSec() > 0) {
      ROS_WARN("TF Cleared for old time");
    }
    {
      boost::mutex::scoped_lock lock(image_mutex_);
      if (msg->encoding.find("bayer") != std::string::npos) {
        original_image_ = cv::Mat(msg->height, msg->width, CV_8UC1,
                                  const_cast<uint8_t*>(&msg->data[0]), msg->step);
      } else {
        try {
          original_image_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
        } catch (cv_bridge::Exception& e) {
          ROS_ERROR("Unable to convert %s image to bgr8", msg->encoding.c_str());
          return;
        }
      }
      // Hang on to message pointer for sake of mouseCb
      last_msg_ = msg;
      redraw();
    }
  }

  void ImageView2::drawImage() {
    if (image_.rows > 0 && image_.cols > 0) {
      redraw();
    }
  }

  void ImageView2::addPoint(int x, int y)
  {
    cv::Point2d p;
    p.x = x;
    p.y = y;
    {
      boost::mutex::scoped_lock lock(point_array_mutex_);
      point_array_.push_back(p);
    }
  }

  void ImageView2::setRegionWindowPoint(int x, int y)
  {
    boost::mutex::scoped_lock lock(point_array_mutex_);
    ROS_DEBUG("setRegionWindowPoint");
    if (selecting_fg_) {
      rect_fg_.x = x;
      rect_fg_.y = y;
    }
    else {
      rect_bg_.x = x;
      rect_bg_.y = y;
    }
  }

  void ImageView2::updateRegionWindowSize(int x, int y)
  {
    ROS_DEBUG("updateRegionWindowPoint");
    boost::mutex::scoped_lock lock(point_array_mutex_);
    if (selecting_fg_) {
      rect_fg_.width = x - rect_fg_.x;
      rect_fg_.height = y - rect_fg_.y;
    }
    else {
      rect_bg_.width = x - rect_bg_.x;
      rect_bg_.height = y - rect_bg_.y;
    }
  }
  
  void ImageView2::addRegionPoint(int x, int y)
  {
    cv::Point2d p;
    p.x = x;
    p.y = y;
    {
      boost::mutex::scoped_lock lock(point_array_mutex_);
      if (selecting_fg_) {
        point_fg_array_.push_back(p);
      }
      else {
        point_bg_array_.push_back(p);
      }
    }
  }

  void ImageView2::clearPointArray()
  {
    boost::mutex::scoped_lock lock(point_array_mutex_);
    point_fg_array_.clear();
    point_bg_array_.clear();
    point_array_.clear();
  }

  void ImageView2::publishPointArray()
  {
    pcl::PointCloud<pcl::PointXY> pcl_cloud;
    for (size_t i = 0; i < point_array_.size(); i++) {
      pcl::PointXY p;
      p.x = point_array_[i].x;
      p.y = point_array_[i].y;
      pcl_cloud.points.push_back(p);
    }
    sensor_msgs::PointCloud2::Ptr ros_cloud(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(pcl_cloud, *ros_cloud);
    ros_cloud->header.stamp = ros::Time::now();
    
    point_array_pub_.publish(ros_cloud);
  }
    
  
  void ImageView2::setMode(KEY_MODE mode)
  {
    mode_ = mode;
  }

  ImageView2::KEY_MODE ImageView2::getMode()
  {
    return mode_;
  }

  bool ImageView2::toggleSelection()
  {
    boost::mutex::scoped_lock lock(point_array_mutex_);
    selecting_fg_ = !selecting_fg_;
    return selecting_fg_;
  }

  void ImageView2::pointArrayToMask(std::vector<cv::Point2d>& points,
                                    cv::Mat& mask)
  {
    cv::Point2d from, to;
    if (points.size() > 1) {
      from = points[0];
      for (size_t i = 1; i < points.size(); i++) {
        to = points[i];
        cv::line(mask, from, to, cv::Scalar(255), 8, 8, 0);
        from = to;
      }
    }
  }

  void ImageView2::publishMonoImage(ros::Publisher& pub,
                                    cv::Mat& image,
                                    const std_msgs::Header& header)
  {
    cv_bridge::CvImage image_bridge(
      header, sensor_msgs::image_encodings::MONO8, image);
    pub.publish(image_bridge.toImageMsg());
  }
  
  void ImageView2::publishForegroundBackgroundMask()
  {
    boost::mutex::scoped_lock lock2(point_array_mutex_);
    cv::Mat foreground_mask
      = cv::Mat::zeros(last_msg_->height, last_msg_->width, CV_8UC1);
    cv::Mat background_mask
      = cv::Mat::zeros(last_msg_->height, last_msg_->width, CV_8UC1);
    if (getMode() == MODE_SELECT_FORE_AND_BACK) {
      pointArrayToMask(point_fg_array_, foreground_mask);
      pointArrayToMask(point_bg_array_, background_mask);
    }
    else if (getMode() == MODE_SELECT_FORE_AND_BACK_RECT) {
      cv::rectangle(foreground_mask, rect_fg_, cv::Scalar(255), CV_FILLED);
      cv::rectangle(background_mask, rect_bg_, cv::Scalar(255), CV_FILLED);
    }
    publishMonoImage(foreground_mask_pub_, foreground_mask, last_msg_->header);
    publishMonoImage(background_mask_pub_, background_mask, last_msg_->header);
  }

  bool ImageView2::isValidMovement(const ros::Time& clicked_time,
                                   const cv::Point2f& start_point,
                                   const cv::Point2f& end_point)
  {
    double dist_px = cv::norm(cv::Mat(start_point), cv::Mat(end_point));
    return clicked_time.toSec() > 0 && dist_px > 3.0;
  }
  
  void ImageView2::mouseCb(int event, int x, int y, int flags, void* param)
  {
    ROS_DEBUG("mouseCB");
    ImageView2 *iv = (ImageView2*)param;
    static ros::Time left_buttondown_time(0);
    switch (event){
    case CV_EVENT_MOUSEMOVE: {
      cv::Point2f Pt_1(window_selection_.x, window_selection_.y);
      cv::Point2f Pt(x, y);
      if (isValidMovement(left_buttondown_time, Pt_1, Pt)) {
        if (iv->getMode() == MODE_RECTANGLE) {
          window_selection_.width  = x - window_selection_.x;
          window_selection_.height = y - window_selection_.y;
        }
        else if (iv->getMode() == MODE_SERIES) {
          iv->addPoint(x, y);
        }
        else if (iv->getMode() == MODE_SELECT_FORE_AND_BACK) {
          iv->addRegionPoint(x, y);
        }
        else if (iv->getMode() == MODE_SELECT_FORE_AND_BACK_RECT) {
          iv->updateRegionWindowSize(x, y);
        }
      }
      {
        // publish the points
        geometry_msgs::PointStamped move_point;
        move_point.header.stamp = ros::Time::now();
        move_point.point.x = x;
        move_point.point.y = y;
        move_point.point.z = 0;
        iv->move_point_pub_.publish(move_point);
      }
      break;
    }
    case CV_EVENT_LBUTTONDOWN:  // click
      left_buttondown_time = ros::Time::now();
      window_selection_.x = x;
      window_selection_.y = y;
      if (iv->getMode() == MODE_SELECT_FORE_AND_BACK_RECT) {
        iv->setRegionWindowPoint(x, y);
      }
      break;
    case CV_EVENT_LBUTTONUP:
      if (iv->getMode() == MODE_SERIES) {
        iv->publishPointArray();
        iv->clearPointArray();
      }
      else if (iv->getMode() == MODE_SELECT_FORE_AND_BACK ||
               iv->getMode() == MODE_SELECT_FORE_AND_BACK_RECT) {
        bool fgp = iv->toggleSelection();
        if (fgp) {
          iv->publishForegroundBackgroundMask();
          //iv->clearPointArray();
        }
      }
      else if (iv->getMode() == MODE_RECTANGLE) {
        cv::Point2f Pt_1(window_selection_.x, window_selection_.y);
        cv::Point2f Pt(x, y);
        if (!isValidMovement(left_buttondown_time, Pt_1, Pt)) {
          geometry_msgs::PointStamped screen_msg;
          screen_msg.point.x = window_selection_.x * resize_x_;
          screen_msg.point.y = window_selection_.y * resize_y_;
          screen_msg.point.z = 0;
          screen_msg.header.stamp = ros::Time::now();
          ROS_INFO("Publish screen point %s (%f %f)", iv->point_pub_.getTopic().c_str(), screen_msg.point.x, screen_msg.point.y);
          iv->point_pub_.publish(screen_msg);
        } else {
          geometry_msgs::PolygonStamped screen_msg;
          screen_msg.polygon.points.resize(2);
          screen_msg.polygon.points[0].x = window_selection_.x * resize_x_;
          screen_msg.polygon.points[0].y = window_selection_.y * resize_y_;
          screen_msg.polygon.points[1].x = (window_selection_.x + window_selection_.width) * resize_x_;
          screen_msg.polygon.points[1].y = (window_selection_.y + window_selection_.height) * resize_y_;
          screen_msg.header.stamp = ros::Time::now();
          ROS_INFO("Publish rectangle point %s (%f %f %f %f)", iv->rectangle_pub_.getTopic().c_str(),
                   screen_msg.polygon.points[0].x, screen_msg.polygon.points[0].y,
                   screen_msg.polygon.points[1].x, screen_msg.polygon.points[1].y);
          iv->rectangle_pub_.publish(screen_msg);
        }
      }
      window_selection_.x = window_selection_.y =
        window_selection_.width = window_selection_.height = 0;
      left_buttondown_time.fromSec(0);
      break;
    case CV_EVENT_RBUTTONDOWN:
    {
      boost::mutex::scoped_lock lock(iv->image_mutex_);
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
    }
    {
      boost::mutex::scoped_lock lock2(iv->image_mutex_);
      iv->drawImage();
    }
    return;
  }

  void ImageView2::pressKey(int key)
  {
    if (key != -1) {
      if (getMode() == MODE_SELECT_FORE_AND_BACK) {
        // only grabcut works here
        switch (key) {
        case 27: {
          boost::mutex::scoped_lock lock(point_array_mutex_);
          point_fg_array_.clear();
          point_bg_array_.clear();
          selecting_fg_ = true;
          break;
        }
        }
      }
      if (getMode() == MODE_SELECT_FORE_AND_BACK_RECT) {
        // only grabcut works here
        switch (key) {
        case 27: {
          boost::mutex::scoped_lock lock(point_array_mutex_);
          rect_fg_.width = rect_fg_.height = 0;
          rect_bg_.width = rect_bg_.height = 0;
          selecting_fg_ = true;
          break;
        }
        }
      }
    }
  }

  void ImageView2::showImage()
  {
    if (use_window) {
      if (!window_initialized_) {
        cv::setMouseCallback(window_name_.c_str(), &ImageView2::mouseCb, this);
        cv::namedWindow(window_name_.c_str(), autosize_ ? CV_WINDOW_AUTOSIZE : 0);
        window_initialized_ = false;
      }
      cv::imshow(window_name_.c_str(), image_);
    }
  }
  
  CvRect ImageView2::window_selection_;
  double ImageView2::resize_x_, ImageView2::resize_y_;
}



int main(int argc, char **argv)
{
  //ros::init(argc, argv, "image_view2", ros::init_options::AnonymousName);
  ros::init(argc, argv, "image_view2");
  ros::NodeHandle n;

  if ( n.resolveName("image") == "/image") {
    ROS_WARN("image_view: image has not been remapped! Typical command-line usage:\n"
             "\t$ ./image_view image:=<image topic> [transport]");
  }
  ros::AsyncSpinner spinner(1);
  image_view2::ImageView2 view(n);
  spinner.start();
  while (ros::ok()) {
    int key = cv::waitKey(1000 / 30);
    view.pressKey(key);
    view.showImage();
  }
  return 0;
}
