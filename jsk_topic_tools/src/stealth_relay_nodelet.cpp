// -*- mode: C++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, JSK Lab
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
/*
 * stealth_relay_nodelet.cpp
 * Author: Furushchev <furushchev@jsk.imi.i.u-tokyo.ac.jp>
 */

#include <jsk_topic_tools/stealth_relay.h>


namespace jsk_topic_tools
{
  void StealthRelay::onInit()
  {
    bool use_multithread;
    ros::param::param<bool>("~use_multithread_callback", use_multithread, true);
    if (use_multithread) {
      NODELET_DEBUG("use multithread callback");
      nh_.reset (new ros::NodeHandle (getMTNodeHandle ()));
      pnh_.reset (new ros::NodeHandle (getMTPrivateNodeHandle ()));
    } else {
      NODELET_DEBUG("use singlethread callback");
      nh_.reset (new ros::NodeHandle (getNodeHandle ()));
      pnh_.reset (new ros::NodeHandle (getPrivateNodeHandle ()));
    }

    subscribed_ = false;
    advertised_ = false;

#ifdef topic_tools_relay_stealth_EXISTS
    NODELET_WARN("This nodelet is deprecated. Use `topic_tools/Relay` with `stealth_mode`");
#endif

    poll_timer_ = pnh_->createTimer(ros::Duration(1.0),
                                    &StealthRelay::timerCallback, this,
                                    /* oneshot= */false, /* autostart= */ false);

    srv_ = boost::make_shared<dynamic_reconfigure::Server<Config> >(*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind(&StealthRelay::configCallback, this, _1, _2);
    srv_->setCallback(f);

    /* To advertise output topic as the same type of input topic,
       first this node subscribes input topic until one message is received. */
    subscribe();
  }

  void StealthRelay::subscribe()
  {
    NODELET_DEBUG("subscribe");
    sub_ = pnh_->subscribe<const ros::MessageEvent<topic_tools::ShapeShifter const>&>
      ("input", queue_size_, &StealthRelay::inputCallback, this);
    subscribed_ = true;
  }

  void StealthRelay::unsubscribe()
  {
    NODELET_DEBUG("unsubscribe");
    sub_.shutdown();
    subscribed_ = false;
  }

  bool StealthRelay::isSubscribed()
  {
    return subscribed_;
  }

  void StealthRelay::configCallback(Config& config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    NODELET_DEBUG("configCallback");
    bool need_resubscribe = (config.queue_size != queue_size_);
    queue_size_ = config.queue_size;

    if (config.monitor_topic.empty()) {
      config.monitor_topic = pnh_->resolveName("input");
    }
    monitor_topic_ = config.monitor_topic;

    if (monitor_rate_ != config.monitor_rate) {
      monitor_rate_ = config.monitor_rate;
      poll_timer_.setPeriod(ros::Duration(monitor_rate_), true);
    }

    if (enable_monitor_ != config.enable_monitor) {
      enable_monitor_ = config.enable_monitor;
      if (enable_monitor_) {
        poll_timer_.start();
      } else {
        poll_timer_.stop();
        subscribe();
      }
    }

    if (isSubscribed() && need_resubscribe) {
      unsubscribe();
      subscribe();
    }
  }

  void StealthRelay::inputCallback(const ros::MessageEvent<topic_tools::ShapeShifter const>& event)
  {
    const boost::shared_ptr<topic_tools::ShapeShifter const>& msg = event.getConstMessage();
    inputCallback(msg);
  }

  void StealthRelay::inputCallback(const boost::shared_ptr<topic_tools::ShapeShifter const>& msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    NODELET_DEBUG("inputCallback");

    if (!advertised_)
    {
      pub_ = msg->advertise(*pnh_, "output", 1);
      advertised_ = true;
      if (enable_monitor_) unsubscribe();
      return;
    }

    pub_.publish(msg);
  }

  void StealthRelay::timerCallback(const ros::TimerEvent& event)
  {
    boost::mutex::scoped_lock lock(mutex_);
    NODELET_DEBUG("timerCallback");

    if (pub_.getNumSubscribers() == 0 && isSubscribed())
    {
      unsubscribe();
      return;
    }

    int subscribing_num = getNumOtherSubscribers(monitor_topic_);
    if (subscribed_ && subscribing_num == 0)
      unsubscribe();
    else if (!subscribed_ && subscribing_num > 0)
      subscribe();
  }

  int StealthRelay::getNumOtherSubscribers(const std::string& name)
  {
    int num = 0;
    XmlRpc::XmlRpcValue req(ros::this_node::getName()), res, data;
    bool ok = ros::master::execute("getSystemState", req, res, data, false);

    XmlRpc::XmlRpcValue& sub_info = data[1];
    for (size_t i = 0; i < sub_info.size(); ++i)
    {
      std::string topic_name = sub_info[i][0];
      if (topic_name == name)
      {
        XmlRpc::XmlRpcValue& subscribers = sub_info[i][1];
        int cnt = 0;
        for (size_t j = 0; j < subscribers.size(); ++j)
        {
          std::string subscriber = subscribers[j];
          if (subscriber != ros::this_node::getName()) ++cnt;
        }
        num = cnt;
        break;
      }
    }
    return num;
  }
}

#include <pluginlib/class_list_macros.h>
typedef jsk_topic_tools::StealthRelay StealthRelay;
PLUGINLIB_EXPORT_CLASS(StealthRelay, nodelet::Nodelet)
