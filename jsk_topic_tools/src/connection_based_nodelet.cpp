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

#include "jsk_topic_tools/connection_based_nodelet.h"
#include "jsk_topic_tools/log_utils.h"

namespace jsk_topic_tools
{
  void ConnectionBasedNodelet::onInit()
  {
    connection_status_ = NOT_SUBSCRIBED;
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
    pnh_->param("always_subscribe", always_subscribe_, false);
    pnh_->param("verbose_connection", verbose_connection_, false);
    if (!verbose_connection_) {
      nh_->param("verbose_connection", verbose_connection_, false);
    }

    // timer to warn when onInitProcess is not called
    pnh_->param("no_warn_on_init_post_process", on_init_post_process_called_, false);
    if (!on_init_post_process_called_) {
      timer_warn_on_init_post_process_called_ = nh_->createWallTimer(
        ros::WallDuration(5),
        &ConnectionBasedNodelet::warnOnInitPostProcessCalledCallback,
        this,
        /*oneshot=*/true);
    }

    // timer to warn when no connection in a few seconds
    ever_subscribed_ = false;
    timer_warn_never_subscribed_ = nh_->createWallTimer(
      ros::WallDuration(5),
      &ConnectionBasedNodelet::warnNeverSubscribedCallback,
      this,
      /*oneshot=*/true);
  }

  void ConnectionBasedNodelet::onInitPostProcess()
  {
    on_init_post_process_called_ = true;
    if (always_subscribe_) {
      boost::mutex::scoped_lock lock(connection_mutex_);
      ever_subscribed_ = true;
      subscribe();
    }
  }

  inline bool ConnectionBasedNodelet::isSubscribed()
  {
    return connection_status_ == SUBSCRIBED;
  }


  void ConnectionBasedNodelet::warnOnInitPostProcessCalledCallback(const ros::WallTimerEvent& event)
  {
    if (!on_init_post_process_called_) {
      NODELET_WARN("[%s] onInitPostProcess is not yet called.", nodelet::Nodelet::getName().c_str());
    }
  }

  void ConnectionBasedNodelet::warnNeverSubscribedCallback(const ros::WallTimerEvent& event)
  {
    if (!ever_subscribed_) {
      NODELET_WARN("'%s' subscribes topics only with child subscribers.", nodelet::Nodelet::getName().c_str());
    }
  }

#if nodelet_VERSION_MINOR > 9 || (nodelet_VERSION_MINOR == 9 && nodelet_VERSION_PATCH >= 11)
  bool ConnectionBasedNodelet::warnNoRemap(const std::vector<std::string> names)
  {
    bool no_warning = true;
    // standalone
    ros::M_string remappings = ros::names::getRemappings();
    // load
    ros::M_string remappings_loaded = getRemappingArgs();
    for (ros::M_string::iterator it = remappings_loaded.begin();
        it != remappings_loaded.end(); it++)
    {
      remappings[it->first] = it->second;
    }
    for (size_t i = 0; i < names.size(); i++)
    {
      std::string resolved_name = ros::names::resolve(/*name=*/names[i],
                                                      /*_remap=*/false);
      if (remappings.find(resolved_name) == remappings.end())
      {
        NODELET_WARN("[%s] '%s' has not been remapped.", getName().c_str(), names[i].c_str());
        no_warning = false;
      }
    }
    return no_warning;
  }
#endif

  void ConnectionBasedNodelet::connectionCallback(const ros::SingleSubscriberPublisher& pub)
  {
    if (verbose_connection_) {
      NODELET_INFO("New connection or disconnection is detected");
    }
    if (!always_subscribe_) {
      boost::mutex::scoped_lock lock(connection_mutex_);
      for (size_t i = 0; i < publishers_.size(); i++) {
        ros::Publisher pub = publishers_[i];
        if (pub.getNumSubscribers() > 0) {
          if (!ever_subscribed_) {
            ever_subscribed_ = true;
          }
          if (connection_status_ != SUBSCRIBED) {
            if (verbose_connection_) {
              NODELET_INFO("Subscribe input topics");
            }
            subscribe();
            connection_status_ = SUBSCRIBED;
          }
          return;
        }
      }
      if (connection_status_ == SUBSCRIBED) {
        if (verbose_connection_) {
          NODELET_INFO("Unsubscribe input topics");
        }
        unsubscribe();
        connection_status_ = NOT_SUBSCRIBED;
      }
    }
  }
  
  void ConnectionBasedNodelet::imageConnectionCallback(
    const image_transport::SingleSubscriberPublisher& pub)
  {
    if (verbose_connection_) {
      NODELET_INFO("New image connection or disconnection is detected");
    }
    if (!always_subscribe_) {
      boost::mutex::scoped_lock lock(connection_mutex_);
      for (size_t i = 0; i < image_publishers_.size(); i++) {
        image_transport::Publisher pub = image_publishers_[i];
        if (pub.getNumSubscribers() > 0) {
          if (!ever_subscribed_) {
            ever_subscribed_ = true;
          }
          if (connection_status_ != SUBSCRIBED) {
            if (verbose_connection_) {
              NODELET_INFO("Subscribe input topics");
            }
            subscribe();
            connection_status_ = SUBSCRIBED;
          }
          return;
        }
      }
      if (connection_status_ == SUBSCRIBED) {
        if (verbose_connection_) {
          NODELET_INFO("Unsubscribe input topics");
        }
        unsubscribe();
        connection_status_ = NOT_SUBSCRIBED;
      }
    }
  }

  void ConnectionBasedNodelet::cameraConnectionCallback(
    const image_transport::SingleSubscriberPublisher& pub)
  {
    cameraConnectionBaseCallback();
  }

  void ConnectionBasedNodelet::cameraInfoConnectionCallback(
    const ros::SingleSubscriberPublisher& pub)
  {
    cameraConnectionBaseCallback();
  }

  void ConnectionBasedNodelet::cameraConnectionBaseCallback()
  {
    if (verbose_connection_) {
      NODELET_INFO("New image connection or disconnection is detected");
    }
    if (!always_subscribe_) {
      boost::mutex::scoped_lock lock(connection_mutex_);
      for (size_t i = 0; i < camera_publishers_.size(); i++) {
        image_transport::CameraPublisher pub = camera_publishers_[i];
        if (pub.getNumSubscribers() > 0) {
          if (!ever_subscribed_) {
            ever_subscribed_ = true;
          }
          if (connection_status_ != SUBSCRIBED) {
            if (verbose_connection_) {
              NODELET_INFO("Subscribe input topics");
            }
            subscribe();
            connection_status_ = SUBSCRIBED;
          }
          return;
        }
      }
      if (connection_status_ == SUBSCRIBED) {
        if (verbose_connection_) {
          NODELET_INFO("Unsubscribe input topics");
        }
        unsubscribe();
        connection_status_ = NOT_SUBSCRIBED;
      }
    }
  }
}
