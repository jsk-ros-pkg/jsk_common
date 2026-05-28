// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2026, JSK Lab
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

/*
 * connection_based_lifecycle_node.cpp
 * Author: Yoshiki Obinata <obinata@jsk.imi.i.u-tokyo.ac.jp>
 */

#include "jsk_topic_tools/connection_based_lifecycle_node.hpp"

#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>

namespace jsk_topic_tools
{

ConnectionBasedLifecycleNode::ConnectionBasedLifecycleNode(const std::string& node_name,
                                                           const rclcpp::NodeOptions& options)
: rclcpp_lifecycle::LifecycleNode(node_name, options), connection_status_(NOT_INITIALIZED),
  always_subscribe_(false), verbose_connection_(false), ever_subscribed_(false),
  on_init_post_process_called_(false)
{
  this->declare_parameter<bool>("always_subscribe", false);
  this->declare_parameter<bool>("verbose_connection", false);
}

void ConnectionBasedLifecycleNode::onInitPostProcess()
{
  on_init_post_process_called_ = true;

  this->get_parameter("always_subscribe", always_subscribe_);
  this->get_parameter("verbose_connection", verbose_connection_);

  // Auto-configure: unconfigured -> inactive
  this->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  if (always_subscribe_) {
    // Auto-activate and subscribe immediately
    this->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
    std::lock_guard<std::mutex> lock(connection_mutex_);
    ever_subscribed_ = true;
    connection_status_ = SUBSCRIBED;
    subscribe();
    RCLCPP_INFO(this->get_logger(), "Always-subscribe mode: subscribed to input topics.");
  }

  // One-shot timer: warn if no subscriber connects within 5 seconds
  timer_warn_never_subscribed_ = this->create_wall_timer(std::chrono::seconds(5), [this]() {
    if (!ever_subscribed_) {
      RCLCPP_WARN(this->get_logger(), "'%s' subscribes topics only with child subscribers.",
                  this->get_name());
    }
    timer_warn_never_subscribed_->cancel();
  });

  RCLCPP_INFO(this->get_logger(), "Configured (always_subscribe=%s, verbose_connection=%s)",
              always_subscribe_ ? "true" : "false", verbose_connection_ ? "true" : "false");
}

// --- Internal lifecycle callbacks ---
// These are simple handlers. on_activate/on_deactivate do NOT lock connection_mutex_
// so that connectionCallback() can safely call trigger_transition() while holding the mutex.

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ConnectionBasedLifecycleNode::on_configure(const rclcpp_lifecycle::State& state)
{
  (void)state;
  connection_status_ = NOT_SUBSCRIBED;
  return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ConnectionBasedLifecycleNode::on_activate(const rclcpp_lifecycle::State& state)
{
  // Activate LifecyclePublishers
  LifecycleNode::on_activate(state);
  return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ConnectionBasedLifecycleNode::on_deactivate(const rclcpp_lifecycle::State& state)
{
  // Deactivate LifecyclePublishers
  LifecycleNode::on_deactivate(state);
  return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ConnectionBasedLifecycleNode::on_cleanup(const rclcpp_lifecycle::State& state)
{
  (void)state;

  std::lock_guard<std::mutex> lock(connection_mutex_);
  if (connection_status_ == SUBSCRIBED) {
    unsubscribe();
  }
  publishers_.clear();
  connection_status_ = NOT_INITIALIZED;
  ever_subscribed_ = false;

  if (timer_warn_never_subscribed_) {
    timer_warn_never_subscribed_->cancel();
    timer_warn_never_subscribed_.reset();
  }

  return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ConnectionBasedLifecycleNode::on_shutdown(const rclcpp_lifecycle::State& state)
{
  (void)state;

  std::lock_guard<std::mutex> lock(connection_mutex_);
  if (connection_status_ == SUBSCRIBED) {
    unsubscribe();
    connection_status_ = NOT_SUBSCRIBED;
  }
  publishers_.clear();

  if (timer_warn_never_subscribed_) {
    timer_warn_never_subscribed_->cancel();
    timer_warn_never_subscribed_.reset();
  }

  return CallbackReturn::SUCCESS;
}

bool ConnectionBasedLifecycleNode::isSubscribed() const { return connection_status_ == SUBSCRIBED; }

void ConnectionBasedLifecycleNode::connectionCallback()
{
  if (!on_init_post_process_called_ || always_subscribe_) {
    return;
  }

  if (verbose_connection_) {
    RCLCPP_INFO(this->get_logger(), "New connection or disconnection detected.");
  }

  std::lock_guard<std::mutex> lock(connection_mutex_);

  // Check if any publisher has subscribers
  for (const auto& pub : publishers_) {
    if (pub->get_subscription_count() > 0) {
      if (!ever_subscribed_) {
        ever_subscribed_ = true;
      }
      if (connection_status_ != SUBSCRIBED) {
        if (verbose_connection_) {
          RCLCPP_INFO(this->get_logger(), "Subscribe input topics.");
        }
        // Auto-activate: inactive -> active (activates LifecyclePublishers)
        if (this->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
          this->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
        }
        subscribe();
        connection_status_ = SUBSCRIBED;
      }
      return;
    }
  }

  // No publisher has any subscriber — unsubscribe if currently subscribed
  if (connection_status_ == SUBSCRIBED) {
    if (verbose_connection_) {
      RCLCPP_INFO(this->get_logger(), "Unsubscribe input topics.");
    }
    unsubscribe();
    connection_status_ = NOT_SUBSCRIBED;
    // Auto-deactivate: active -> inactive (deactivates LifecyclePublishers)
    if (this->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      this->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
    }
  }
}

} // namespace jsk_topic_tools
