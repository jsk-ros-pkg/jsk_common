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
 * connection_based_lifecycle_node.hpp
 * Author: Yoshiki Obinata <obinata@jsk.imi.i.u-tokyo.ac.jp>
 */

#ifndef CONNECTION_BASED_LIFECYCLE_NODE_HPP_
#define CONNECTION_BASED_LIFECYCLE_NODE_HPP_

#include <memory>
#include <mutex>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

namespace jsk_topic_tools
{

enum ConnectionStatus
{
  NOT_INITIALIZED,
  NOT_SUBSCRIBED,
  SUBSCRIBED
};

/** @brief
 * ROS2 Lifecycle Node base class that automatically subscribes/unsubscribes
 * input topics according to the subscription status of advertised output topics.
 *
 * Lifecycle transitions are managed internally. From outside, the node appears
 * as active when output topics have subscribers, and inactive otherwise.
 * Subclasses do NOT need to be aware of lifecycle at all.
 *
 * Subclass usage (mirrors ConnectionBasedNodelet from ROS1):
 *   1. In the constructor, create publishers with advertise<T>().
 *   2. Call onInitPostProcess() at the end of the constructor.
 *   3. Implement subscribe() to create input subscriptions.
 *   4. Implement unsubscribe() to destroy input subscriptions.
 *
 * Ported from ROS1 ConnectionBasedNodelet.
 */
class ConnectionBasedLifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit ConnectionBasedLifecycleNode(
    const std::string & node_name,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  virtual ~ConnectionBasedLifecycleNode() = default;

protected:
  // --- User-facing interface (mirrors ConnectionBasedNodelet) ---

  /** @brief
   * Post processing of initialization.
   * Call this at the end of your constructor after creating publishers
   * with advertise(). This auto-configures the lifecycle node and,
   * if always_subscribe is true, also auto-activates and subscribes.
   */
  void onInitPostProcess();

  /** @brief
   * Called when at least one output publisher gains a subscriber.
   * Subclass should create input subscriptions here.
   */
  virtual void subscribe() = 0;

  /** @brief
   * Called when all output publishers lose their subscribers.
   * Subclass should destroy input subscriptions here.
   */
  virtual void unsubscribe() = 0;

  /** @brief
   * Returns true when this node is currently subscribed to input topics.
   */
  virtual bool isSubscribed() const;

  /** @brief
   * Advertise a topic and register a matched_callback to track subscriber
   * connections. Use this instead of create_publisher().
   *
   * The return type is LifecyclePublisher, but it can be stored as either:
   *   rclcpp::Publisher<T>::SharedPtr  (lifecycle-unaware)
   *   rclcpp_lifecycle::LifecyclePublisher<T>::SharedPtr  (lifecycle-aware)
   *
   * @param topic Topic name to advertise.
   * @param qos   QoS profile for the publisher.
   * @return SharedPtr to the created LifecyclePublisher.
   */
  template<typename MessageT>
  typename rclcpp_lifecycle::LifecyclePublisher<MessageT>::SharedPtr
  advertise(const std::string & topic, const rclcpp::QoS & qos)
  {
    std::lock_guard<std::mutex> lock(connection_mutex_);

    rclcpp::PublisherOptions pub_options;
    pub_options.event_callbacks.matched_callback = [this](rclcpp::MatchedInfo & info) {
      (void)info;
      connectionCallback();
    };

    auto pub = this->create_publisher<MessageT>(topic, qos, pub_options);
    publishers_.push_back(pub);
    return pub;
  }

  /** @brief
   * Convenience overload accepting a queue size (depth) integer.
   *
   * @param topic      Topic name to advertise.
   * @param queue_size Queue depth for the publisher.
   * @return SharedPtr to the created LifecyclePublisher.
   */
  template<typename MessageT>
  typename rclcpp_lifecycle::LifecyclePublisher<MessageT>::SharedPtr
  advertise(const std::string & topic, size_t queue_size)
  {
    return advertise<MessageT>(topic, rclcpp::QoS(queue_size));
  }

  /** @brief Mutex protecting publishers_, connection_status_, and subscribe/unsubscribe calls. */
  std::mutex connection_mutex_;

  /** @brief List of tracked publishers (type-erased). */
  std::vector<rclcpp::PublisherBase::SharedPtr> publishers_;

  /** @brief Current connection status. */
  ConnectionStatus connection_status_;

  /** @brief If true, always subscribe regardless of output subscribers. */
  bool always_subscribe_;

  /** @brief If true, log connection/disconnection events. */
  bool verbose_connection_;

  /** @brief Flag indicating if any subscriber has ever connected. */
  bool ever_subscribed_;

private:
  // --- Internal lifecycle management (not for subclass use) ---

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State & state) override;

  /** @brief
   * Callback invoked when a subscriber connects to or disconnects from
   * any of the tracked publishers. Manages lifecycle transitions and
   * calls subscribe()/unsubscribe() automatically.
   */
  void connectionCallback();

  /** @brief Flag to ignore connectionCallback before onInitPostProcess is called. */
  bool on_init_post_process_called_;

  /** @brief One-shot timer for warning when no subscriber connects for a while. */
  rclcpp::TimerBase::SharedPtr timer_warn_never_subscribed_;
};

}  // namespace jsk_topic_tools

#endif  // CONNECTION_BASED_LIFECYCLE_NODE_HPP_
