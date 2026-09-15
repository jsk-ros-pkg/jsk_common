// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, JSK Lab
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

#include "image_view2.h"

#if ROS_VERSION_MAJOR != 1
#include <thread>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("image_view2");

  if (node->get_node_base_interface()->resolve_topic_or_service_name("image", false) == "/image") {
    RCLCPP_WARN(node->get_logger(),
                "image_view: image has not been remapped! Typical command-line usage:\n"
                "\t$ ros2 run image_view2 image_view2 --ros-args -r image:=<image topic>");
  }
  image_view2::ImageView2 view(node);
  // ROS2 has no AsyncSpinner; spin on a background thread instead, same
  // shape as ROS1's spinner+main-thread-GUI-loop split (OpenCV's HighGUI
  // calls below must run on the main thread).
  std::thread spin_thread([node]() { rclcpp::spin(node); });
  while (rclcpp::ok()) {
    int key = cv::waitKey(1000 / 30);
    view.pressKey(key);
    view.showImage();
  }
  spin_thread.join();
  return 0;
}
#else

int main(int argc, char **argv)
{
  //ros::init(argc, argv, "image_view2", ros::init_options::AnonymousName);
  ros::init(argc, argv, "image_view2");
  ros::NodeHandle n;

  if ( n.resolveName("image") == "/image") {
    ROS_WARN("image_view: image has not been remapped! Typical command-line usage:\n"
             "\t$ ./image_view image:=<image topic> [transport]");
  }
  ros::AsyncSpinner spinner(1);
  image_view2::ImageView2 view(n);
  spinner.start();
  while (ros::ok()) {
    int key = cv::waitKey(1000 / 30);
    view.pressKey(key);
    view.showImage();
  }
  return 0;
}
#endif

