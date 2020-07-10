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
 * stealth_relay.h
 * Author: Furushchev <furushchev@jsk.imi.i.u-tokyo.ac.jp>
 */


#ifndef STEALTH_RELAY_H__
#define STEALTH_RELAY_H__

#include <boost/thread.hpp>
#include <nodelet/nodelet.h>
#include <topic_tools/shape_shifter.h>

#include <dynamic_reconfigure/server.h>
#include <jsk_topic_tools/StealthRelayConfig.h>


namespace jsk_topic_tools
{
  class StealthRelay : public nodelet::Nodelet
  {
    typedef StealthRelayConfig Config;
  protected:
    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();
    virtual bool isSubscribed();
    virtual void configCallback(Config& config, uint32_t level);
    virtual void inputCallback(const ros::MessageEvent<topic_tools::ShapeShifter const>& event);
    virtual void inputCallback(const boost::shared_ptr<topic_tools::ShapeShifter const>& msg);
    virtual void timerCallback(const ros::TimerEvent& event);
    virtual int getNumOtherSubscribers(const std::string& name);

    boost::mutex mutex_;
    boost::shared_ptr<ros::NodeHandle> nh_, pnh_;
    boost::shared_ptr<dynamic_reconfigure::Server<Config> > srv_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    ros::Timer poll_timer_;
    std::string monitor_topic_;
    double monitor_rate_;
    int queue_size_;
    bool enable_monitor_;
    bool subscribed_;
    bool advertised_;
  };
}

#endif // STEALTH_RELAY_H__
