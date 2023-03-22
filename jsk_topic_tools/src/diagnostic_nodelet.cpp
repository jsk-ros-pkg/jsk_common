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

#include "jsk_topic_tools/diagnostic_nodelet.h"
#include <sstream>
namespace jsk_topic_tools
{

  DiagnosticNodelet::DiagnosticNodelet():
    name_(getName()) {}

  DiagnosticNodelet::DiagnosticNodelet(const std::string& name):
    name_(name)
  {

  }
  
  void DiagnosticNodelet::onInit()
  {
    ConnectionBasedNodelet::onInit();
    previous_checked_connection_status_ = NOT_SUBSCRIBED;
    diagnostic_updater_.reset(
      new TimeredDiagnosticUpdater(*pnh_, ros::Duration(1.0)));
    diagnostic_updater_->setHardwareID(getName());
    diagnostic_updater_->add(
      getName(),
#if __cplusplus < 201400L
      boost::bind(
        &DiagnosticNodelet::updateDiagnostic,
        this,
        _1)
#else
      [this](auto& stat){ updateDiagnostic(stat); }
#endif
     );

    bool use_warn;
    nh_->param("/diagnostic_nodelet/use_warn", use_warn, false);
    if (pnh_->hasParam("use_warn")) {
      pnh_->getParam("use_warn", use_warn);
    }
    if (use_warn)
    {
      diagnostic_error_level_ = diagnostic_msgs::DiagnosticStatus::WARN;
    } else {
      diagnostic_error_level_ = diagnostic_msgs::DiagnosticStatus::ERROR;
    }

    double vital_rate;
    pnh_->param("vital_rate", vital_rate, 1.0);
    vital_checker_.reset(
      new jsk_topic_tools::VitalChecker(1 / vital_rate));
    diagnostic_updater_->start();
  }

  void DiagnosticNodelet::updateDiagnostic(
    diagnostic_updater::DiagnosticStatusWrapper &stat)
  {
    if (connection_status_ == SUBSCRIBED) {
      if (previous_checked_connection_status_ != connection_status_) {
        // Poke when start subscribing.
        vital_checker_->poke();
      }
      if (vital_checker_->isAlive()) {
        stat.summary(diagnostic_msgs::DiagnosticStatus::OK,
                     getName() + " running");
      }
      else {
        jsk_topic_tools::addDiagnosticErrorSummary(
          getName(), vital_checker_, stat, diagnostic_error_level_);
      }
    }
    else {
      stat.summary(diagnostic_msgs::DiagnosticStatus::OK,
                   getName() + " is not subscribed");
    }
    std::stringstream topic_names;
    for (size_t i = 0; i < publishers_.size(); i++) {
      if (i == publishers_.size() - 1) {
        topic_names << publishers_[i].getTopic();
      }
      else {
        topic_names << publishers_[i].getTopic() << ", ";
      }
    }
    stat.add("watched topics", topic_names.str());
    for (size_t i = 0; i < publishers_.size(); i++) {
      stat.add(publishers_[i].getTopic(),
               (boost::format("%d subscribers") %
                publishers_[i].getNumSubscribers()).str());
    }
    previous_checked_connection_status_ = connection_status_;
  }
}
