// -*- mode: C++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, JSK Lab
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
 * synchronized_throttle_nodelet.cpp
 * Author: furushchev <furushchev@jsk.imi.i.u-tokyo.ac.jp>
 */

#include <jsk_topic_tools/synchronized_throttle.h>

namespace jsk_topic_tools
{

void SynchronizedThrottle::onInit()
{
  // nodelet handle initialization
  bool use_multithread;
  ros::param::param<bool>("~use_multithread_callback", use_multithread, true);
  if (use_multithread) {
    NODELET_DEBUG("use multithread callback");
    nh_.reset (new ros::NodeHandle(getMTNodeHandle()));
    pnh_.reset (new ros::NodeHandle(getMTPrivateNodeHandle()));
  } else {
    NODELET_DEBUG("use singlethread callback");
    nh_.reset (new ros::NodeHandle(getNodeHandle()));
    pnh_.reset (new ros::NodeHandle(getPrivateNodeHandle()));
  }

  // load parameters
  pnh_->param("enable_warning", enable_warning_, true);
  pnh_->param("topics", input_topics_, std::vector<std::string>());
  const size_t n_topics = input_topics_.size();
  subscribed_ = false;
  advertised_ = false;

  srv_ = boost::make_shared<dynamic_reconfigure::Server<Config> >(*pnh_);
  dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind(&SynchronizedThrottle::configCallback, this, _1, _2);
  srv_->setCallback(f);

  // message_filter supports 2~8 input topics
  if (n_topics > MAX_SYNC_NUM)
  {
    NODELET_FATAL_STREAM("Found more than " << MAX_SYNC_NUM << " input topics which is not supported");
    return;
  } else if (n_topics < 2) {
    NODELET_FATAL("Found less than 2 input topics which is not supported");
    return;
  }

  NODELET_DEBUG_STREAM(n_topics << " input topics");

  // subscribe to identify topic types
  check_sub_.resize(n_topics);
  sub_.resize(n_topics);
  pub_.resize(n_topics);
  for (size_t i = 0; i < n_topics; ++i)
  {
    check_sub_[i] = pnh_->subscribe<topic_tools::ShapeShifterStamped>(
        input_topics_[i], 1,
        boost::bind(&SynchronizedThrottle::checkCallback, this, _1, i));
    sub_[i].reset(new message_filters::Subscriber<topic_tools::ShapeShifterStamped>());
  }

  if (enable_warning_)
  {
    check_timer_ = pnh_->createWallTimer(
        ros::WallDuration(5), &SynchronizedThrottle::checkAdvertisedTimerCallback,
        this, /* oneshot= */false);
  }
}

SynchronizedThrottle::~SynchronizedThrottle() {
  // This fixes the following error on shutdown of the nodelet:
  // terminate called after throwing an instance of
  // 'boost::exception_detail::clone_impl<boost::exception_detail::error_info_injector<boost::lock_error> >'
  //     what():  boost: mutex lock failed in pthread_mutex_lock: Invalid argument
  // Also see ros/ros_comm#720 .
  if (approximate_sync_) {
    async_.reset();
  } else {
    sync_.reset();
  }
}

void SynchronizedThrottle::configCallback(Config &config, uint32_t level)
{
  boost::mutex::scoped_lock lock(mutex_);

  update_rate_ = config.update_rate;

  if (use_wall_time_ != config.use_wall_time)
  {
    use_wall_time_ = config.use_wall_time;
    if (use_wall_time_) last_stamp_.fromSec(ros::WallTime::now().toSec());
    else                 last_stamp_ = ros::Time::now();
  }

  if (config.suffix.empty())
  {
    NODELET_ERROR("parameter suffix cannot be empty");
    if (suffix_.empty()) config.suffix = "throttled";
    else                 config.suffix = suffix_;
  }

  if (approximate_sync_ != config.approximate_sync ||
      queue_size_ != config.queue_size ||
      suffix_ != config.suffix)
  {
    approximate_sync_ = config.approximate_sync;
    queue_size_ = config.queue_size;
    suffix_ = config.suffix;
    if (subscribed_)
    {
      unsubscribe();
      subscribe();
    }
  }
}

void SynchronizedThrottle::checkAdvertisedTimerCallback(const ros::WallTimerEvent& event)
{
  for (size_t i = 0; i < pub_.size(); ++i)
  {
    if (!pub_[i])
    {
      NODELET_WARN_STREAM(input_topics_[i] << " is not yet published");
    }
  }
  if (advertised_)
  {
    NODELET_INFO("All topics are now published and synchronized");
    check_timer_.stop();
  }
}

void SynchronizedThrottle::subscribe()
{
  NODELET_DEBUG("subscribe");

  const size_t n_topics = input_topics_.size();

  for (size_t i = 0; i < n_topics; ++i)
  {
    sub_[i]->subscribe(*pnh_, input_topics_[i], 1);
  }
  if (n_topics < MAX_SYNC_NUM)
  {
    sub_[0]->registerCallback(
        boost::bind(&SynchronizedThrottle::fillNullMessage, this, _1));
  }

  if (approximate_sync_)
  {
    async_ = boost::make_shared<message_filters::Synchronizer<AsyncPolicy> >(queue_size_);

    switch (n_topics)
    {
      case 2:
        async_->connectInput(*sub_[0], *sub_[1], null_, null_,
                             null_, null_, null_, null_);
        break;
      case 3:
        async_->connectInput(*sub_[0], *sub_[1], *sub_[2], null_,
                             null_, null_, null_, null_);
        break;
      case 4:
        async_->connectInput(*sub_[0], *sub_[1], *sub_[2], *sub_[3],
                             null_, null_, null_, null_);
        break;
      case 5:
        async_->connectInput(*sub_[0], *sub_[1], *sub_[2], *sub_[3],
                             *sub_[4], null_, null_, null_);
        break;
      case 6:
        async_->connectInput(*sub_[0], *sub_[1], *sub_[2], *sub_[3],
                             *sub_[4], *sub_[5], null_, null_);
        break;
      case 7:
        async_->connectInput(*sub_[0], *sub_[1], *sub_[2], *sub_[3],
                             *sub_[4], *sub_[5], *sub_[6], null_);
        break;
      case 8:
        async_->connectInput(*sub_[0], *sub_[1], *sub_[2], *sub_[3],
                             *sub_[4], *sub_[5], *sub_[6], *sub_[7]);
        break;
      default:
        NODELET_FATAL("Unhandled error");
        return;
    }
    async_->registerCallback(
        boost::bind(&SynchronizedThrottle::inputCallback, this,
                    _1, _2, _3, _4, _5, _6, _7, _8));
  } else {
    sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(queue_size_);

    switch (n_topics)
    {
      case 2:
        sync_->connectInput(*sub_[0], *sub_[1], null_, null_,
                            null_, null_, null_, null_);
        break;
      case 3:
        sync_->connectInput(*sub_[0], *sub_[1], *sub_[2], null_,
                            null_, null_, null_, null_);
        break;
      case 4:
        sync_->connectInput(*sub_[0], *sub_[1], *sub_[2], *sub_[3],
                            null_, null_, null_, null_);
        break;
      case 5:
        sync_->connectInput(*sub_[0], *sub_[1], *sub_[2], *sub_[3],
                            *sub_[4], null_, null_, null_);
        break;
      case 6:
        sync_->connectInput(*sub_[0], *sub_[1], *sub_[2], *sub_[3],
                            *sub_[4], *sub_[5], null_, null_);
        break;
      case 7:
        sync_->connectInput(*sub_[0], *sub_[1], *sub_[2], *sub_[3],
                            *sub_[4], *sub_[5], *sub_[6], null_);
        break;
      case 8:
        sync_->connectInput(*sub_[0], *sub_[1], *sub_[2], *sub_[3],
                            *sub_[4], *sub_[5], *sub_[6], *sub_[7]);
        break;
      default:
        NODELET_FATAL("Unhandled error");
        return;
    }
    sync_->registerCallback(
        boost::bind(&SynchronizedThrottle::inputCallback, this,
                    _1, _2, _3, _4, _5, _6, _7, _8));
  }
}

void SynchronizedThrottle::unsubscribe()
{
  NODELET_DEBUG("unsubscribe");

  for (size_t i = 0; i < sub_.size(); ++i)
  {
    sub_[i]->unsubscribe();
  }
}

bool SynchronizedThrottle::isSubscribed() const
{
  return subscribed_;
}

void SynchronizedThrottle::checkCallback(
    const topic_tools::ShapeShifterStamped::ConstPtr& msg,
    const size_t index)
{
  boost::mutex::scoped_lock lock(mutex_);

  NODELET_DEBUG_STREAM("check callback: " << index);
  NODELET_DEBUG_STREAM(" name: " << input_topics_[index]);
  NODELET_DEBUG_STREAM(" type: " << msg->getDataType());
  NODELET_DEBUG_STREAM(" md5: " << msg->getMD5Sum());

  check_sub_[index].shutdown();

  // advertise
  ros::SubscriberStatusCallback connect_cb
      = boost::bind(&SynchronizedThrottle::connectCb, this);
  ros::SubscriberStatusCallback disconnect_cb
      = boost::bind(&SynchronizedThrottle::disconnectCb, this);
  std::string output_topic = input_topics_[index] + "/" + suffix_;
  ros::AdvertiseOptions options(
      output_topic, 1,
      msg->getMD5Sum(), msg->getDataType(), msg->getMessageDefinition(),
      connect_cb, disconnect_cb);
  options.latch = false;
  pub_[index] = pnh_->advertise(options);

  // check if all are advertised
  bool all_advertised = true;
  for (size_t i = 0; i < pub_.size(); ++i)
  {
    if (!pub_[i]) all_advertised = false;
  }
  if (all_advertised)
  {
    NODELET_DEBUG("All Advertised");
    advertised_ = true;
    if (!subscribed_)
    {
      for (size_t i = 0; i < pub_.size(); ++i)
      {
        if (pub_[i].getNumSubscribers() > 0) {
          subscribe();
          subscribed_ = true;
          break;
        }
      }
    }
  }
}

void SynchronizedThrottle::connectCb()
{
  boost::mutex::scoped_lock lock(mutex_);

  NODELET_DEBUG("connectCb");

  if (advertised_ && !subscribed_)
  {
    for (size_t i = 0; i < pub_.size(); ++i)
    {
      if (pub_[i].getNumSubscribers() > 0) {
        subscribe();
        subscribed_ = true;
        break;
      }
    }
  }
}

void SynchronizedThrottle::disconnectCb()
{
  // need to check https://github.com/ros/ros_comm/issues/158 ?
  boost::mutex::scoped_lock lock(mutex_);

  NODELET_DEBUG("disconnectCb");

  if (subscribed_)
  {
    bool need_unsubscribe = true;
    for (size_t i = 0; i < pub_.size(); ++i)
    {
      if (pub_[i].getNumSubscribers() > 0)
      {
        need_unsubscribe = false;
        break;
      }
    }
    if (need_unsubscribe)
    {
      unsubscribe();
      subscribed_ = false;
    }
  }
}

void SynchronizedThrottle::fillNullMessage(
    const topic_tools::ShapeShifterStamped::ConstPtr& msg)
{
  NODELET_DEBUG("fill null message");
  null_.add(msg);
}

void SynchronizedThrottle::inputCallback(
    const topic_tools::ShapeShifterStamped::ConstPtr& msg0,
    const topic_tools::ShapeShifterStamped::ConstPtr& msg1,
    const topic_tools::ShapeShifterStamped::ConstPtr& msg2,
    const topic_tools::ShapeShifterStamped::ConstPtr& msg3,
    const topic_tools::ShapeShifterStamped::ConstPtr& msg4,
    const topic_tools::ShapeShifterStamped::ConstPtr& msg5,
    const topic_tools::ShapeShifterStamped::ConstPtr& msg6,
    const topic_tools::ShapeShifterStamped::ConstPtr& msg7)
{
  boost::mutex::scoped_lock lock(mutex_);
  NODELET_DEBUG("input callback");

  ros::Time now;
  if (use_wall_time_) now.fromSec(ros::WallTime::now().toSec());
  else                now = ros::Time::now();

  // detect time jump back
  if (last_stamp_ > now)
  {
    NODELET_WARN("Detected jump back in time. last_stamp_ is overwritten.");
    last_stamp_ = now;
  }

  // throttle
  if (update_rate_ <= 0.0 ||
      (now - last_stamp_).toSec() < 1.0 / update_rate_)
    return;

  // publish
  topic_tools::ShapeShifterStamped::ConstPtr msgs[] =
      {msg0, msg1, msg2, msg3, msg4, msg5, msg6, msg7};

  for (size_t i = 0; i < pub_.size(); ++i)
  {
    if (pub_[i].getNumSubscribers() > 0)
    {
      pub_[i].publish(msgs[i]);
    }
  }
  last_stamp_ = now;
}

} // jsk_topic_tools

#include <pluginlib/class_list_macros.h>
typedef jsk_topic_tools::SynchronizedThrottle SynchronizedThrottle;
PLUGINLIB_EXPORT_CLASS(SynchronizedThrottle, nodelet::Nodelet)
