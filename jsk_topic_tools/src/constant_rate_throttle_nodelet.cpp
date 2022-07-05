#include <jsk_topic_tools/constant_rate_throttle_nodelet.h>

namespace jsk_topic_tools
{
  void ConstantRateThrottle::onInit()
  {
    pnh_ = this->getPrivateNodeHandle();
    subscribing_ = false;
    advertised_ = false;
    msg_cached_ = boost::shared_ptr<topic_tools::ShapeShifter>(new topic_tools::ShapeShifter());

    srv_ = boost::make_shared<dynamic_reconfigure::Server<Config> >(pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f = boost::bind(&ConstantRateThrottle::configCallback, this, _1, _2);
    srv_->setCallback(f);

    sub_.reset(new ros::Subscriber(
                 pnh_.subscribe<topic_tools::ShapeShifter>(
                   "input", 1,
                   &ConstantRateThrottle::inCallback,
                   this,
                   th_)));
  }

  void ConstantRateThrottle::configCallback(Config& config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    update_rate_ = config.update_rate;
    duration_message_valid_ = ros::Duration(config.duration_message_valid);
    if ( this->isLoopAlive() ) {
        this->stopPublishLoop();
        this->startPublishLoop(update_rate_);
    }
  }

  void ConstantRateThrottle::connectionCallback(const ros::SingleSubscriberPublisher& pub)
  {
    boost::mutex::scoped_lock lock(mutex_);
    if (pub_.getNumSubscribers() > 0) {
      if (!subscribing_) {
        sub_.reset(new ros::Subscriber(
                     pnh_.subscribe<topic_tools::ShapeShifter>(
                       "input", 1,
                       &ConstantRateThrottle::inCallback,
                       this,
                       th_)));
        subscribing_ = true;
        this->startPublishLoop(update_rate_);
      }
    }
    else {      // No subscribers, nodelet can unsubscribe input topic
      if (subscribing_) {
        sub_->shutdown();
        subscribing_ = false;
        this->stopPublishLoop();
      }
    }
  }

  void ConstantRateThrottle::inCallback(const boost::shared_ptr<topic_tools::ShapeShifter const>& msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    if (!advertised_) {
        sub_->shutdown();
        ros::SubscriberStatusCallback connect_cb = boost::bind(&ConstantRateThrottle::connectionCallback, this, _1);
        ros::AdvertiseOptions opts("output", 1,
                                   msg->getMD5Sum(),
                                   msg->getDataType(),
                                   msg->getMessageDefinition(),
                                   connect_cb,
                                   connect_cb);
        advertised_ = true;
        pub_ = pnh_.advertise(opts);
    }

    *msg_cached_ = *msg;
    time_cached_ = ros::Time::now();
  }

  void ConstantRateThrottle::publishMessage(const ros::TimerEvent&)
  {
    ros::Time current_time = ros::Time::now();
    if ( not msg_cached_ ) {
      ROS_WARN("No message is Cached .");
    } else if ( current_time - time_cached_ < duration_message_valid_ ) {
      pub_.publish(msg_cached_);
    } else {
      ROS_WARN("Cached message is too old.");
    }
  }

  bool ConstantRateThrottle::isLoopAlive()
  {
      return timer_publish_.isValid() and timer_publish_.hasStarted();
  }

  void ConstantRateThrottle::startPublishLoop(double loop_rate)
  {
    if ( not timer_publish_.isValid() ) {
        timer_publish_ = pnh_.createTimer(
                ros::Duration(1.0/update_rate_),
                &ConstantRateThrottle::publishMessage,
                this
                );
    } else {
        timer_publish_.setPeriod(ros::Duration(1.0/update_rate_));
        timer_publish_.start();
    }
  }

  void ConstantRateThrottle::stopPublishLoop()
  {
    timer_publish_.stop();
  }

}


#include <pluginlib/class_list_macros.h>
typedef jsk_topic_tools::ConstantRateThrottle ConstantRateThrottle;
PLUGINLIB_EXPORT_CLASS(ConstantRateThrottle, nodelet::Nodelet)
