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
#include <pluginlib/class_list_macros.h>
#include "jsk_topic_tools/relay_nodelet.h"

namespace jsk_topic_tools
{

  void Relay::onInit()
  {
    advertised_ = false;
    subscribing_ = false;
    pnh_ = getPrivateNodeHandle();
    sub_ = pnh_.subscribe<topic_tools::ShapeShifter>("input", 1,
                                                     &Relay::inputCallback, this, th_);
  }
  
  void Relay::inputCallback(const boost::shared_ptr<topic_tools::ShapeShifter const>& msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    if (!advertised_) {
      // this block is called only once
      // in order to advertise topic.
      // we need to subscribe at least one message
      // in order to detect message definition.
      ros::SubscriberStatusCallback connect_cb
        = boost::bind( &Relay::connectCb, this);
      ros::SubscriberStatusCallback disconnect_cb
        = boost::bind( &Relay::disconnectCb, this);
      ros::AdvertiseOptions opts("output", 1,
                                 msg->getMD5Sum(),
                                 msg->getDataType(),
                                 msg->getMessageDefinition(),
                                 connect_cb,
                                 disconnect_cb);
      opts.latch = false;
      pub_ = pnh_.advertise(opts);
      advertised_ = true;
      // shutdown subscriber
      sub_.shutdown();
    }
    else if (pub_.getNumSubscribers() > 0) {
      pub_.publish(msg);
    }
  }

  void Relay::connectCb()
  {
    boost::mutex::scoped_lock lock(mutex_);
    NODELET_DEBUG("connectCB");
    if (advertised_) {
      if (pub_.getNumSubscribers() > 0) {
        if (!subscribing_) {
          NODELET_DEBUG("suscribe");
          sub_ = pnh_.subscribe<topic_tools::ShapeShifter>("input", 1,
                                                           &Relay::inputCallback, this, th_);
          subscribing_ = true;
        }
      }
    }
  }

  void Relay::disconnectCb()
  {
    boost::mutex::scoped_lock lock(mutex_);
    NODELET_DEBUG("disconnectCb");
    if (advertised_) {
      if (pub_.getNumSubscribers() == 0) {
        if (subscribing_) {
          NODELET_DEBUG("disconnect");
          sub_.shutdown();
          subscribing_ = false;
        }
      }
    }
  }
  
}

typedef jsk_topic_tools::Relay Relay;
PLUGINLIB_EXPORT_CLASS(Relay, nodelet::Nodelet)
