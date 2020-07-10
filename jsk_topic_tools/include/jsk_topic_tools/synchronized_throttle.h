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
 * synchronized_throttle_nodelet.h
 * Author: furushchev <furushchev@jsk.imi.i.u-tokyo.ac.jp>
 */


#ifndef SYNCHRONIZED_THROTTLE_NODELET_H__
#define SYNCHRONIZED_THROTTLE_NODELET_H__

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <dynamic_reconfigure/server.h>
#include <jsk_topic_tools/SynchronizedThrottleConfig.h>
#include <message_filters/pass_through.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#include <nodelet/nodelet.h>
#include <std_msgs/Header.h>
#include <topic_tools/shape_shifter.h>

namespace topic_tools
{
class ShapeShifterStamped : public ShapeShifter
{
 public:
  typedef boost::shared_ptr<ShapeShifterStamped> Ptr;
  typedef boost::shared_ptr<ShapeShifterStamped const> ConstPtr;
  void fillHeader() {
    uint8_t buf[size()];
    ros::serialization::OStream stream(buf, size());
    write(stream);
    header.seq = ((uint32_t*)buf)[0];
    header.stamp.sec = ((uint32_t*)buf)[1];
    header.stamp.nsec = ((uint32_t*)buf)[2];
  }

  std_msgs::Header header;
};
}

namespace ros
{
namespace message_traits
{
template<> struct HasHeader<topic_tools::ShapeShifterStamped> : public TrueType {};
template<> struct HasHeader<const topic_tools::ShapeShifterStamped> : public TrueType {};
template<>
struct Header<topic_tools::ShapeShifterStamped,
              typename boost::enable_if<HasHeader<topic_tools::ShapeShifterStamped> >::type>
{
  static std_msgs::Header* pointer(topic_tools::ShapeShifterStamped& m) { return &m.header; }
  static std_msgs::Header const* pointer(const topic_tools::ShapeShifterStamped& m) { return &m.header; }
};
template<>
struct TimeStamp<topic_tools::ShapeShifterStamped,
                 typename boost::enable_if<HasHeader<topic_tools::ShapeShifterStamped> >::type>
{
  static ros::Time* pointer(typename boost::remove_const<topic_tools::ShapeShifterStamped>::type &m) { return &m.header.stamp; }
  static ros::Time const* pointer(const topic_tools::ShapeShifterStamped& m) { return &m.header.stamp; }
  static ros::Time value(const topic_tools::ShapeShifterStamped& m) { return m.header.stamp; }
};
template<>
struct MD5Sum<topic_tools::ShapeShifterStamped>
{
  static const char* value(const topic_tools::ShapeShifterStamped& m) { return m.getMD5Sum().c_str(); }
  static const char* value() { return "*"; }
};
template<>
struct DataType<topic_tools::ShapeShifterStamped>
{
  static const char* value(const topic_tools::ShapeShifterStamped& m) { return m.getDataType().c_str(); }
  static const char* value() { return "*"; }
};

} // message_traits


namespace serialization
{

template<>
struct Serializer<topic_tools::ShapeShifterStamped>
{
  template<typename Stream>
  inline static void write(Stream& stream, const topic_tools::ShapeShifterStamped& m) {
    m.write(stream);
  }

  template<typename Stream>
  inline static void read(Stream& stream, topic_tools::ShapeShifterStamped& m)
  {
    m.read(stream);
    m.fillHeader();
  }

  inline static uint32_t serializedLength(const topic_tools::ShapeShifterStamped& m) {
    return m.size();
  }
};
template<>
struct PreDeserialize<topic_tools::ShapeShifterStamped>
{
  static void notify(const PreDeserializeParams<topic_tools::ShapeShifterStamped>& params)
  {
    std::string md5      = (*params.connection_header)["md5sum"];
    std::string datatype = (*params.connection_header)["type"];
    std::string msg_def  = (*params.connection_header)["message_definition"];
    std::string latching  = (*params.connection_header)["latching"];

    params.message->morph(md5, datatype, msg_def, latching);
  }
};
} // serialization
} // ros

namespace jsk_topic_tools
{
class SynchronizedThrottle : public nodelet::Nodelet
{
 public:
  typedef message_filters::sync_policies::ExactTime<
   topic_tools::ShapeShifterStamped,
   topic_tools::ShapeShifterStamped,
   topic_tools::ShapeShifterStamped,
   topic_tools::ShapeShifterStamped,
   topic_tools::ShapeShifterStamped,
   topic_tools::ShapeShifterStamped,
   topic_tools::ShapeShifterStamped,
   topic_tools::ShapeShifterStamped> SyncPolicy;

  typedef message_filters::sync_policies::ApproximateTime<
   topic_tools::ShapeShifterStamped,
   topic_tools::ShapeShifterStamped,
   topic_tools::ShapeShifterStamped,
   topic_tools::ShapeShifterStamped,
   topic_tools::ShapeShifterStamped,
   topic_tools::ShapeShifterStamped,
   topic_tools::ShapeShifterStamped,
   topic_tools::ShapeShifterStamped> AsyncPolicy;

  typedef jsk_topic_tools::SynchronizedThrottleConfig Config;

  static const int MAX_SYNC_NUM = 8;

 protected:
  virtual ~SynchronizedThrottle();
  virtual void onInit();
  virtual void configCallback(Config &config, uint32_t level);
  virtual void subscribe();
  virtual void unsubscribe();
  virtual bool isSubscribed() const;
  virtual void checkCallback(
      const topic_tools::ShapeShifterStamped::ConstPtr &msg,
      const size_t index);
  virtual void connectCb();
  virtual void disconnectCb();
  virtual void checkAdvertisedTimerCallback(const ros::WallTimerEvent& event);

  virtual void fillNullMessage(
      const topic_tools::ShapeShifterStamped::ConstPtr& msg);
  virtual void inputCallback(
      const topic_tools::ShapeShifterStamped::ConstPtr& msg0,
      const topic_tools::ShapeShifterStamped::ConstPtr& msg1,
      const topic_tools::ShapeShifterStamped::ConstPtr& msg2,
      const topic_tools::ShapeShifterStamped::ConstPtr& msg3,
      const topic_tools::ShapeShifterStamped::ConstPtr& msg4,
      const topic_tools::ShapeShifterStamped::ConstPtr& msg5,
      const topic_tools::ShapeShifterStamped::ConstPtr& msg6,
      const topic_tools::ShapeShifterStamped::ConstPtr& msg7);

  boost::mutex mutex_;
  boost::shared_ptr<ros::NodeHandle> nh_, pnh_;
  boost::shared_ptr<dynamic_reconfigure::Server<Config> > srv_;
  ros::WallTimer check_timer_;
  boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> > sync_;
  boost::shared_ptr<message_filters::Synchronizer<AsyncPolicy> > async_;
  std::vector<ros::Subscriber> check_sub_;
  std::vector<boost::shared_ptr<message_filters::Subscriber<topic_tools::ShapeShifterStamped> > > sub_;
  message_filters::PassThrough<topic_tools::ShapeShifterStamped> null_;
  std::vector<ros::Publisher> pub_;

  std::vector<std::string> input_topics_;
  std::string suffix_;
  ros::Time last_stamp_;
  bool subscribed_;
  bool advertised_;
  bool enable_warning_;
  bool use_wall_time_;
  bool approximate_sync_;
  int queue_size_;
  double update_rate_;
};
}


#endif // SYNCHRONIZED_THROTTLE_NODELET_H__
