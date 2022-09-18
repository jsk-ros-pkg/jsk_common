#ifndef CONSTANT_RATE_THROTTLE_NODELET_H_
#define CONSTANT_RATE_THROTTLE_NODELET_H_

#include <boost/shared_ptr.hpp>
#include <nodelet/nodelet.h>

#include <dynamic_reconfigure/server.h>
#include <jsk_topic_tools/ConstantRateThrottleConfig.h>
#include <topic_tools/shape_shifter.h>
#include <mutex>

namespace jsk_topic_tools
{
  class ConstantRateThrottle : public nodelet::Nodelet
  {
  public:
    typedef ConstantRateThrottleConfig Config;
    virtual void onInit();
    virtual void configCallback(Config& config, uint32_t level);
    virtual void publishMessage(const ros::TimerEvent&);
    virtual bool isLoopAlive();
    virtual void startPublishLoop(double loop_rate);
    virtual void stopPublishLoop();
    virtual void connectionCallback(const ros::SingleSubscriberPublisher& pub);
    virtual void inCallback(const boost::shared_ptr<topic_tools::ShapeShifter const>& msg);
  protected:
    typedef boost::shared_ptr<ros::Subscriber> SubscriberPtr;
    boost::shared_ptr<dynamic_reconfigure::Server<Config> > srv_;
    SubscriberPtr sub_;
    ros::Publisher pub_;
    ros::TransportHints th_;
    ros::NodeHandle pnh_;
    ros::Timer timer_publish_;
    bool timer_started_;

    boost::mutex mutex_;

    bool subscribing_;
    bool advertised_;

    double update_rate_;
    ros::Duration duration_message_valid_;

    ros::Time time_cached_;
    boost::shared_ptr<topic_tools::ShapeShifter> msg_cached_;
  };
}

#endif
