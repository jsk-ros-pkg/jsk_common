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


#ifndef CONNECTION_BASED_NODELET_H_
#define CONNECTION_BASED_NODELET_H_

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <boost/thread.hpp>
#include <image_transport/image_transport.h>
#include "jsk_topic_tools/log_utils.h"

namespace jsk_topic_tools
{
  /** @brief
   * Enum to represent connection status.
   */
  enum ConnectionStatus
  {
    NOT_INITIALIZED,
    NOT_SUBSCRIBED,
    SUBSCRIBED
  };
  
  /** @brief
   * Nodelet to automatically subscribe/unsubscribe
   * topics according to subscription of advertised topics.
   *
   * It's important not to subscribe topic if no output is required.
   *
   * In order to watch advertised topics, need to use advertise template method.
   * And create subscribers in subscribe() and shutdown them in unsubscribed().
   *
   */
  class ConnectionBasedNodelet: public nodelet::Nodelet
  {
  public:
    ConnectionBasedNodelet(): subscribed_(false) { }
  protected:
    
    /** @brief
     * Initialize nodehandles nh_ and pnh_. Subclass should call
     * this method in its onInit method
     */
    virtual void onInit();


    /** @brief
     * Post processing of initialization of nodelet.
     * You need to call this method in order to use always_subscribe
     * feature.
     */
    virtual void onInitPostProcess();
    
    /** @brief
     * callback function which is called when new subscriber connects or disconnects
     */
    virtual void connectionCallback(const ros::SingleSubscriberPublisher& pub);

    /** @brief
     * callback function which is called when new subscriber come for image
     * publisher
     */
    virtual void imageConnectionCallback(
      const image_transport::SingleSubscriberPublisher& pub);

    /** @brief
     * callback function which is called when new subscriber come for camera
     * image publisher
     */
    virtual void cameraConnectionCallback(
      const image_transport::SingleSubscriberPublisher& pub);
    
    /** @brief
     * callback function which is called when new subscriber come for
     * camera info publisher
     */
    virtual void cameraInfoConnectionCallback(
      const ros::SingleSubscriberPublisher& pub);
    
    /** @brief
     * callback function which is called when new subscriber come for camera
     * image publisher or camera info publisher.
     * This function is called from cameraConnectionCallback
     * or cameraInfoConnectionCallback.
     */
    virtual void cameraConnectionBaseCallback();

    /** @brief
     * callback function which is called when walltimer
     * duration run out.
     */
    virtual void warnOnInitPostProcessCalledCallback(const ros::WallTimerEvent& event);

    /** @brief
     * callback function which is called when walltimer
     * duration run out.
     */
    virtual void warnNeverSubscribedCallback(const ros::WallTimerEvent& event);

    /** @brief
     * This method is called when publisher is subscribed by other
     * nodes. 
     * Set up subscribers in this method.
     */
    virtual void subscribe() = 0;

    /** @brief
     * This method is called when publisher is unsubscribed by other
     * nodes.
     * Shut down subscribers in this method.
     */
    virtual void unsubscribe() = 0;

    /** @brief
     * Returns true when this nodelet subscribes topics, false otherwise.
     */
    virtual bool isSubscribed();

    /** @brief warn if there are expected remappings.
    *
    * @param[in] names Names which are expected to remapped.
    * @return false if there is at least a topic which is not remapped, else true;
    */
    virtual bool warnNoRemap(const std::vector<std::string> names);

    /** @brief
     * Advertise a topic and watch the publisher. Publishers which are
     * created by this method.
     * It automatically reads latch boolean parameter from nh and
     * publish topic with appropriate latch parameter.
     *
     * @param nh NodeHandle.
     * @param topic topic name to advertise.
     * @param queue_size queue size for publisher.
     * @return Publisher for the advertised topic.
     */
    template<class T> ros::Publisher
    advertise(ros::NodeHandle& nh, std::string topic, int queue_size)
    {
      bool latch;
      nh.param("latch", latch, false);
      return advertise<T>(nh, topic, queue_size, latch);
    }

    /** @brief
     * Advertise a topic and watch the publisher. Publishers which are
     * created by this method.
     *
     * @param nh NodeHandle.
     * @param topic topic name to advertise.
     * @param queue_size queue size for publisher.
     * @param latch set true if latch topic publication.
     * @return Publisher for the advertised topic.
     */
    template<class T> ros::Publisher
    advertise(ros::NodeHandle& nh,
              std::string topic, int queue_size, bool latch)
    {
      boost::mutex::scoped_lock lock(connection_mutex_);
      ros::SubscriberStatusCallback connect_cb
#if __cplusplus < 201400L
        = boost::bind(&ConnectionBasedNodelet::connectionCallback, this, _1);
#else
        = [this](auto& pub){ connectionCallback(pub); };
#endif
      ros::SubscriberStatusCallback disconnect_cb
#if __cplusplus < 201400L
        = boost::bind(&ConnectionBasedNodelet::connectionCallback, this, _1);
#else
        = [this](auto& pub){ connectionCallback(pub); };
#endif
      ros::Publisher ret = nh.advertise<T>(topic, queue_size,
                                           connect_cb,
                                           disconnect_cb,
                                           ros::VoidConstPtr(),
                                           latch);
      publishers_.push_back(ret);

      return ret;
    }

    image_transport::Publisher
    advertiseImage(ros::NodeHandle& nh,
                   image_transport::ImageTransport& it,
                   const std::string& topic,
                   int queue_size)
    {
      NODELET_WARN("advertiseImage with ImageTransport is deprecated");
      return advertiseImage(nh, topic, queue_size);
    }
    
    /** @brief
     * Advertise an image topic and watch the publisher. Publishers which are
     * created by this method.
     * It automatically reads latch boolean parameter from nh and the publisher
     * publishes topic with appropriate latch parameter.
     *
     * @param nh NodeHandle.
     * @param topic topic name to advertise.
     * @param queue_size queue size for publisher.
     * @return Publisher for the advertised topic.
     */
    image_transport::Publisher
    advertiseImage(ros::NodeHandle& nh,
                   const std::string& topic,
                   int queue_size)
    {
      bool latch;
      nh.param("latch", latch, false);
      return advertiseImage(nh, topic, queue_size, latch);
    }

    /** @brief
     * Advertise an image topic and watch the publisher. Publishers which are
     * created by this method.
     *
     * @param nh NodeHandle.
     * @param topic topic name to advertise.
     * @param queue_size queue size for publisher.
     * @param latch set true if latch topic publication.
     * @return Publisher for the advertised topic.
     */
    image_transport::Publisher
    advertiseImage(ros::NodeHandle& nh,
                   const std::string& topic,
                   int queue_size,
                   bool latch)
    {
      boost::mutex::scoped_lock lock(connection_mutex_);
      image_transport::SubscriberStatusCallback connect_cb
#if __cplusplus < 201400L
        = boost::bind(&ConnectionBasedNodelet::imageConnectionCallback,
                      this, _1);
#else
        = [this](auto& pub){ imageConnectionCallback(pub); };
#endif
      image_transport::SubscriberStatusCallback disconnect_cb
#if __cplusplus < 201400L
        = boost::bind(&ConnectionBasedNodelet::imageConnectionCallback,
                      this, _1);
#else
        = [this](auto& pub){ imageConnectionCallback(pub); };
#endif
      image_transport::Publisher pub = image_transport::ImageTransport(nh).advertise(
        topic, 1,
        connect_cb,
        disconnect_cb,
        ros::VoidPtr(),
        latch);
      image_publishers_.push_back(pub);
      return pub;
    }


    image_transport::CameraPublisher
    advertiseCamera(ros::NodeHandle& nh,
                    image_transport::ImageTransport& it,
                    const std::string& topic,
                    int queue_size)
    {
      NODELET_WARN("advertiseCamera with ImageTransport is deprecated");
      return advertiseCamera(nh, topic, queue_size);
    }


    /** @brief
     * Advertise an image and camerainfo topic and watch the publisher.
     * It automatically reads latch boolean parameter from nh and the publisher
     * publishes topic with appropriate latch parameter.
     *
     * @param nh NodeHandle.
     * @param topic topic name to advertise.
     * @param queue_size queue size for publisher.
     * @return Publisher for the advertised topic.
     */
    image_transport::CameraPublisher
    advertiseCamera(ros::NodeHandle& nh,
                    const std::string& topic,
                    int queue_size)
    {
      bool latch;
      nh.param("latch", latch, false);
      return advertiseCamera(nh, topic, queue_size, latch);
    }

    /** @brief
     * Advertise an image and camerainfo topic and watch the publisher.
     *
     * @param nh NodeHandle.
     * @param topic topic name to advertise.
     * @param queue_size queue size for publisher.
     * @param latch set true if latch topic publication.
     * @return Publisher for the advertised topic.
     */
    image_transport::CameraPublisher
    advertiseCamera(ros::NodeHandle& nh,
                    const std::string& topic,
                    int queue_size,
                    bool latch)
    {
      boost::mutex::scoped_lock lock(connection_mutex_);
      image_transport::SubscriberStatusCallback connect_cb
#if __cplusplus < 201400L
        = boost::bind(&ConnectionBasedNodelet::cameraConnectionCallback,
                      this, _1);
#else
        = [this](auto& pub){ cameraConnectionCallback(pub); };
#endif
      image_transport::SubscriberStatusCallback disconnect_cb
#if __cplusplus < 201400L
        = boost::bind(&ConnectionBasedNodelet::cameraConnectionCallback,
                      this, _1);
#else
        = [this](auto& pub){ cameraConnectionCallback(pub); };
#endif
      ros::SubscriberStatusCallback info_connect_cb
#if __cplusplus < 201400L
        = boost::bind(&ConnectionBasedNodelet::cameraInfoConnectionCallback,
                      this, _1);
#else
        = [this](auto& pub){ cameraInfoConnectionCallback(pub); };
#endif
      ros::SubscriberStatusCallback info_disconnect_cb
#if __cplusplus < 201400L
        = boost::bind(&ConnectionBasedNodelet::cameraInfoConnectionCallback,
                      this, _1);
#else
        = [this](auto& pub){ cameraInfoConnectionCallback(pub); };
#endif
      image_transport::CameraPublisher
        pub = image_transport::ImageTransport(nh).advertiseCamera(
          topic, 1,
          connect_cb, disconnect_cb,
          info_connect_cb, info_disconnect_cb,
          ros::VoidPtr(),
          latch);
      camera_publishers_.push_back(pub);
      return pub;
    }
    
    /** @brief
     * mutex to call subscribe() and unsubscribe() in
     * critical section.
     */
    boost::mutex connection_mutex_;
    
    /** @brief
     * List of watching publishers
     */
    std::vector<ros::Publisher> publishers_;

    /** @brief
     * List of watching image publishers
     */
    std::vector<image_transport::Publisher> image_publishers_;

    /** @brief
     * List of watching camera publishers
     */
    std::vector<image_transport::CameraPublisher> camera_publishers_;

    /** @brief
     * Shared pointer to nodehandle.
     */
    boost::shared_ptr<ros::NodeHandle> nh_;

    /** @brief
     * Shared pointer to private nodehandle.
     */
    boost::shared_ptr<ros::NodeHandle> pnh_;

    /** @brief
     * WallTimer instance for warning about no connection.
     */
    ros::WallTimer timer_warn_never_subscribed_;

    /** @brief
     * A flag to check if any publisher is already subscribed
     * or not.
     */
    bool subscribed_;

    /** @brief
     * A flag to check if the node has been ever subscribed
     * or not.
     */
    bool ever_subscribed_;

    /** @brief
     * A flag to disable watching mechanism and always subscribe input 
     * topics. It can be specified via ~always_subscribe parameter.
     */
    bool always_subscribe_;

    /** @brief
     * Status of connection
     */
    ConnectionStatus connection_status_;

    /** @brief
     * true if `~verbose_connection` or `verbose_connection` parameter is true.
     */
    bool verbose_connection_;

    /** @brief
     * Never warning on not calling onInitPostProcess if true
     */
    bool on_init_post_process_called_;

    /** @brief
     * WallTimer instance for warning about no connection.
     */
    ros::WallTimer timer_warn_on_init_post_process_called_;
  private:
    
  };
}

#endif
