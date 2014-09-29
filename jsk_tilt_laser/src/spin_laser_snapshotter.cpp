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

#include <ros/ros.h>

// Services
#include "laser_assembler/AssembleScans2.h"

// Messages
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/JointState.h"

#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
/***
 * This uses the point_cloud_assembler's build_cloud service call to grab all the scans/clouds between two tilt-laser shutters
 * params
 *  * "~target_frame_id" (string) - This is the frame that the scanned data transformed into.  The
 *                                  output clouds are also published in this frame.
 *  * "~num_skips" (int)          - If set to N>0, then the snapshotter will skip N signals before
 *                                  requesting a new snapshot. This will make the snapshots be N times
 *                                  larger. Default 0 - no skipping.
 */

namespace spin_laser_snapshotter
{

class SpinLaserSnapshotter
{

public:
  ros::NodeHandle n_;
  ros::NodeHandle private_ns_;
  ros::Publisher pub_;
  ros::Subscriber sub_;

  ros::Time prev_signal_;
  double prev_angle_;

  bool first_time_;
  bool negative_direction_;
  int num_skips_;
  int num_skips_left_;
  double rate_;
  std::string fixed_frame_;
  std::string spindle_frame_;
  std::string motor_frame_;
  tf::TransformListener* listener_;
  ros::Timer timer_;
  SpinLaserSnapshotter() : private_ns_("~")
  {
    prev_signal_.fromNSec(0);

    pub_ = n_.advertise<sensor_msgs::PointCloud2> ("full_cloud2", 1);

    private_ns_.param("num_skips", num_skips_, 0);
    num_skips_left_=num_skips_;

    prev_angle_ = -1;
    first_time_ = true;
    private_ns_.param("negative_direction", negative_direction_, false);
    ROS_INFO_STREAM("negative_direction: " << negative_direction_);
    bool use_tf;
    private_ns_.param("use_tf", use_tf, false);
    if (!use_tf) {
      sub_ = n_.subscribe("joint_states", 40, &SpinLaserSnapshotter::scannerSignalCallback, this);
    }
    else {
      listener_ = new tf::TransformListener();
      private_ns_.param("tf_polling_rate", rate_, 30.0);
      private_ns_.param("spindle_frame", spindle_frame_, std::string("multisense/spindle"));
      private_ns_.param("motor_frame", motor_frame_, std::string("multisense/motor"));
      timer_ = private_ns_.createTimer(
        ros::Duration(1.0 / rate_), boost::bind(
          &SpinLaserSnapshotter::timerCallback, this, _1));
    }
  }

  ~SpinLaserSnapshotter()
  {

  }

  void timerCallback(const ros::TimerEvent& event)
  {
    // only if it is possible to transform from motor_frame_ -> spindle_frame_
    if (listener_->waitForTransform(motor_frame_, spindle_frame_, event.current_real,
                                    ros::Duration(1 / rate_))) {
      tf::StampedTransform transform;
      listener_->lookupTransform(spindle_frame_, motor_frame_, event.current_real, transform);
      // compute quaternion into eigen
      Eigen::Affine3d t;
      tf::transformTFToEigen(transform, t);
      double yaw   = atan2(t(1,0), t(0,0));
      sensor_msgs::JointState js;
      js.header.stamp = transform.stamp_;
      js.position.push_back(yaw);
      const sensor_msgs::JointState js_const (js);
      scannerSignalCallback(boost::make_shared<sensor_msgs::JointState>(js_const));
    }
  }
  
  void scannerSignalCallback(const sensor_msgs::JointStateConstPtr& js)
  {
    double theta = js->position[0];
    if (negative_direction_) {
      theta = - theta;
    }
    double ang = fmod(theta, 2 * M_PI);

    // ROS_DEBUG("ang = %lf, prev_angle = %lf, %lf", ang, prev_angle_, prev_signal_.toSec());

    if ( prev_angle_ < 0 ) {
      prev_angle_ = ang;
      return;
    }
    if ((ang - prev_angle_) >= - M_PI) {
      prev_angle_ = ang;
      return;
    }

    if (prev_signal_.toSec() == 0.0) {
      first_time_ = true;
    }

    ros::Time stmp = js->header.stamp;
    if (first_time_)
    {
      prev_signal_ = stmp;
      first_time_ = false;
    }
    else
    {
      if (num_skips_ > 0)
      {
        if (num_skips_left_ > 0)
        {
          num_skips_left_ -= 1;
          return;
        }
        else
        {
          num_skips_left_ = num_skips_;
        }
      }

      laser_assembler::AssembleScans2::Request req;
      laser_assembler::AssembleScans2::Response res;

      req.begin = prev_signal_;
      req.end   = stmp;
      if (!ros::service::call("assemble_scans2", req, res))
        ROS_ERROR("Failed to call service on point cloud assembler or laser scan assembler.");

      pub_.publish(res.cloud);
      ROS_DEBUG("Snapshotter::Published Cloud size=%u", res.cloud.width * res.cloud.height);

      prev_signal_ = stmp;
      prev_angle_ = -1;
    }
  }
};
}

using namespace spin_laser_snapshotter;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "spin_laser_snapshotter");
  SpinLaserSnapshotter snapshotter ;
  ros::spin();
  return 0;
}
