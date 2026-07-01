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
 * string_relay_lifecycle_node.cpp
 * Author: Yoshiki Obinata <obinata@jsk.imi.i.u-tokyo.ac.jp>
 *
 * A simple node that inherits from ConnectionBasedLifecycleNode.
 * Relays std_msgs/String from ~/input to ~/output.
 * No lifecycle awareness needed — just advertise, subscribe, unsubscribe.
 */

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/string.hpp>

#include "jsk_topic_tools/connection_based_lifecycle_node.hpp"

namespace jsk_topic_tools
{

class StringRelayLifecycleNode : public ConnectionBasedLifecycleNode
{
public:
  explicit StringRelayLifecycleNode(const rclcpp::NodeOptions & options)
  : ConnectionBasedLifecycleNode("string_relay_lifecycle", options)
  {
    pub_ = advertise<std_msgs::msg::String>("~/output", 10);
    onInitPostProcess();
  }

protected:
  void subscribe() override
  {
    sub_ = this->create_subscription<std_msgs::msg::String>(
      "~/input", 10,
      std::bind(&StringRelayLifecycleNode::callback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Subscribed to input topic.");
  }

  void unsubscribe() override
  {
    sub_.reset();
    RCLCPP_INFO(this->get_logger(), "Unsubscribed from input topic.");
  }

private:
  void callback(const std_msgs::msg::String::SharedPtr msg)
  {
    pub_->publish(*msg);
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

}  // namespace jsk_topic_tools

RCLCPP_COMPONENTS_REGISTER_NODE(jsk_topic_tools::StringRelayLifecycleNode)
