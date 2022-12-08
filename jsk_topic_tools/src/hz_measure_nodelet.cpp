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
 *   * Neither the name of the JSK Lab nor the names of its
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

#include <limits>
#include <boost/format.hpp>
#include <pluginlib/class_list_macros.h>
#include "jsk_topic_tools/hz_measure_nodelet.h"

#include "std_msgs/Float32.h"

namespace jsk_topic_tools
{
  void HzMeasure::onInit()
  {
    pnh_ = getPrivateNodeHandle();
    if (pnh_.getParam("message_num", average_message_num_) &&
        pnh_.getParam("measure_time", measure_time_)) {
      NODELET_WARN("Both ~measure_time and ~message_num are given. Prioritize ~measure_time.");
      average_message_num_ = -1.0;
    }
    else if (pnh_.getParam("measure_time", measure_time_)) {
      average_message_num_ = -1.0;  // disable average_message_num_
      if (measure_time_ < 0.0) {
        NODELET_ERROR("~measure_time should be greater than 0.0. Set 1.0 as default.");
        measure_time_ = 1.0;
      }
    }
    else if (pnh_.getParam("message_num", average_message_num_)) {
      measure_time_ = -1.0;  // diable measure_time_
      if (average_message_num_ < 0) {
        NODELET_ERROR("~message_num should be greater than 0. Set 10 as default.");
        average_message_num_ = 10;
      }
    }
    if (!pnh_.getParam("warning_hz", warning_hz_)) {
      warning_hz_ = -1;
    }
    bool use_warn = false;
    if (pnh_.hasParam("use_warn")) {
      pnh_.getParam("use_warn", use_warn);
    }
    if (use_warn) {
      diagnostic_error_level_ = diagnostic_msgs::DiagnosticStatus::WARN;
    } else {
      diagnostic_error_level_ = diagnostic_msgs::DiagnosticStatus::ERROR;
    }

    diagnostic_updater_.reset(new TimeredDiagnosticUpdater(pnh_, ros::Duration(1.0)));
    diagnostic_updater_->setHardwareID(getName());
    diagnostic_updater_->add(getName(),
                             boost::bind(&HzMeasure::updateDiagnostic,
                                         this, _1));
    diagnostic_updater_->start();

    hz_pub_ = pnh_.advertise<std_msgs::Float32>("output", 1);
    sub_ = pnh_.subscribe<topic_tools::ShapeShifter>("input", 1,
                                                     &HzMeasure::inputCallback, this);
  }

  void HzMeasure::popBufferQueue() {
    ros::Time now = ros::Time::now();
    while (!buffer_.empty()
           && ((measure_time_ > 0 && measure_time_ < (now - buffer_.front()).toSec())
               || (average_message_num_ > 0 && average_message_num_ < buffer_.size()))) {
      buffer_.pop();
    }
  }

  double HzMeasure::calculateHz() {
    double hz = -1.0;
    popBufferQueue();
    if (average_message_num_ > 0) {
      if (buffer_.size() == average_message_num_) {
        ros::Time now = ros::Time::now();
        ros::Time oldest = buffer_.front();
        double whole_time = (now - oldest).toSec();
        double average_time = whole_time / buffer_.size();
        hz = 1.0 / average_time;
      }
    } else {
      double time_width = measure_time_;
      if (buffer_.size() > 0) {
        ros::Time now = ros::Time::now();
        time_width = std::min(measure_time_, std::max((now - buffer_.front()).toSec(), 0.0001));
      }
      hz = std::max(int(buffer_.size() - 1), 0) / time_width;
    }
    return hz;
  }

  void HzMeasure::inputCallback(const boost::shared_ptr<topic_tools::ShapeShifter const>& msg)
  {
    ros::Time now = ros::Time::now();
    buffer_.push(now);
    double hz = calculateHz();
    if (hz > 0.0) {
      std_msgs::Float32 output;
      output.data = hz;
      hz_pub_.publish(output);
    } else {
      NODELET_DEBUG("there is no enough messages yet");
    }
  }

  void HzMeasure::updateDiagnostic(
    diagnostic_updater::DiagnosticStatusWrapper &stat)
  {
    double hz = calculateHz();
    if (hz > 0.0) {
      if (hz > warning_hz_) {
        stat.summary(diagnostic_msgs::DiagnosticStatus::OK,
                     (boost::format("%s is running at %.2f hz.")
                      % getName() % hz).str());
      } else {
        stat.summary(diagnostic_error_level_,
                     (boost::format("%s is running at %.2f hz.")
                      % getName() % hz).str());
      }
    } else {
      stat.summary(diagnostic_error_level_,
                   (boost::format("%s is waiting input topic.")
                    % getName()).str());
    }
  }
}

typedef jsk_topic_tools::HzMeasure HzMeasure;
PLUGINLIB_EXPORT_CLASS(HzMeasure, nodelet::Nodelet)
