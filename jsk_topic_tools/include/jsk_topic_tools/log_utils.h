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


#ifndef JSK_TOPIC_TOOLS_LOG_UTILS_H_
#define JSK_TOPIC_TOOLS_LOG_UTILS_H_

#include <ros/ros.h>
#include <nodelet/nodelet.h>


namespace jsk_topic_tools
{

  /** @brief warn if there are expected remappings.
   *
   * @param[in] names Names which are expected to remapped.
   * @return false if there is at least a topic which is not remapped, else true;
   */
  bool warnNoRemap(const std::vector<std::string> names);


  /** @brief Get only function name from __PRETTY_FUNCTION__
   *
   * @param[in] name std::string from which only function name is extracted
   * @return std::string function name which is extracted
   */
  const std::string getFunctionName(const std::string &name);

}

#define JSK_TOPIC_TOOLS_DEPRECATED_JSK_NODELET_XXX() \
  ROS_WARN_ONCE("DEPRECATION WARNING: JSK_NODELET_XXX log macros are deprecated, and please use NODELET_XXX instead." \
                " (See https://github.com/jsk-ros-pkg/jsk_common/issues/1461)");
#define JSK_TOPIC_TOOLS_DEPRECATED_JSK_ROS_XXX() \
  ROS_WARN_ONCE("DEPRECATION WARNING: JSK_ROS_XXX log utils are deprecated, and please use ROS_XXX instead." \
                " (See https://github.com/jsk-ros-pkg/jsk_common/issues/1461)");

#define JSK_NODELET_DEBUG(str,...) \
  JSK_TOPIC_TOOLS_DEPRECATED_JSK_NODELET_XXX(); \
  NODELET_DEBUG("[%s] " str, jsk_topic_tools::getFunctionName(__PRETTY_FUNCTION__).c_str(), ##__VA_ARGS__)
#define JSK_NODELET_INFO(str,...) \
  JSK_TOPIC_TOOLS_DEPRECATED_JSK_NODELET_XXX(); \
  NODELET_INFO("[%s] " str, jsk_topic_tools::getFunctionName(__PRETTY_FUNCTION__).c_str(), ##__VA_ARGS__)
#define JSK_NODELET_WARN(str,...) \
  JSK_TOPIC_TOOLS_DEPRECATED_JSK_NODELET_XXX(); \
  NODELET_WARN("[%s] " str, jsk_topic_tools::getFunctionName(__PRETTY_FUNCTION__).c_str(), ##__VA_ARGS__)
#define JSK_NODELET_ERROR(str,...) \
  JSK_TOPIC_TOOLS_DEPRECATED_JSK_NODELET_XXX(); \
  NODELET_ERROR("[%s] " str, jsk_topic_tools::getFunctionName(__PRETTY_FUNCTION__).c_str(), ##__VA_ARGS__)
#define JSK_NODELET_FATAL(str,...) \
  JSK_TOPIC_TOOLS_DEPRECATED_JSK_NODELET_XXX(); \
  NODELET_FATAL("[%s] " str, jsk_topic_tools::getFunctionName(__PRETTY_FUNCTION__).c_str(), ##__VA_ARGS__)

#define JSK_NODELET_DEBUG_STREAM(...) \
  JSK_TOPIC_TOOLS_DEPRECATED_JSK_NODELET_XXX(); \
  NODELET_DEBUG_STREAM("[" << jsk_topic_tools::getFunctionName(__PRETTY_FUNCTION__).c_str() << "] " << __VA_ARGS__)
#define JSK_NODELET_INFO_STREAM(...) \
  JSK_TOPIC_TOOLS_DEPRECATED_JSK_NODELET_XXX(); \
  NODELET_INFO_STREAM("[" << jsk_topic_tools::getFunctionName(__PRETTY_FUNCTION__).c_str() << "] " << __VA_ARGS__)
#define JSK_NODELET_WARN_STREAM(...) \
  JSK_TOPIC_TOOLS_DEPRECATED_JSK_NODELET_XXX(); \
  NODELET_WARN_STREAM("[" << jsk_topic_tools::getFunctionName(__PRETTY_FUNCTION__).c_str() << "] " << __VA_ARGS__)
#define JSK_NODELET_ERROR_STREAM(...) \
  JSK_TOPIC_TOOLS_DEPRECATED_JSK_NODELET_XXX(); \
  NODELET_ERROR_STREAM("[" << jsk_topic_tools::getFunctionName(__PRETTY_FUNCTION__).c_str() << "] " << __VA_ARGS__)
#define JSK_NODELET_FATAL_STREAM(...) \
  JSK_TOPIC_TOOLS_DEPRECATED_JSK_NODELET_XXX(); \
  NODELET_FATAL_STREAM("[" << jsk_topic_tools::getFunctionName(__PRETTY_FUNCTION__).c_str() << "] " << __VA_ARGS__)

#define JSK_NODELET_DEBUG_THROTTLE(rate, str, ...) \
  JSK_TOPIC_TOOLS_DEPRECATED_JSK_NODELET_XXX(); \
  NODELET_DEBUG_THROTTLE(rate, "[%s] " str, jsk_topic_tools::getFunctionName(__PRETTY_FUNCTION__).c_str(), ##__VA_ARGS__)
#define JSK_NODELET_INFO_THROTTLE(rate, str, ...) \
  JSK_TOPIC_TOOLS_DEPRECATED_JSK_NODELET_XXX(); \
  NODELET_INFO_THROTTLE(rate, "[%s] " str, jsk_topic_tools::getFunctionName(__PRETTY_FUNCTION__).c_str(), ##__VA_ARGS__)
#define JSK_NODELET_WARN_THROTTLE(rate, str, ...) \
  JSK_TOPIC_TOOLS_DEPRECATED_JSK_NODELET_XXX(); \
  NODELET_WARN_THROTTLE(rate, "[%s] " str, jsk_topic_tools::getFunctionName(__PRETTY_FUNCTION__).c_str(), ##__VA_ARGS__)
#define JSK_NODELET_ERROR_THROTTLE(rate, str, ...) \
  JSK_TOPIC_TOOLS_DEPRECATED_JSK_NODELET_XXX(); \
  NODELET_ERROR_THROTTLE(rate, "[%s] " str, jsk_topic_tools::getFunctionName(__PRETTY_FUNCTION__).c_str(), ##__VA_ARGS__)
#define JSK_NODELET_FATAL_THROTTLE(rate, str, ...) \
  JSK_TOPIC_TOOLS_DEPRECATED_JSK_NODELET_XXX(); \
  NODELET_FATAL_THROTTLE(rate, "[%s] " str, jsk_topic_tools::getFunctionName(__PRETTY_FUNCTION__).c_str(), ##__VA_ARGS__)

#define JSK_NODELET_DEBUG_STREAM_THROTTLE(rate,...) \
  JSK_TOPIC_TOOLS_DEPRECATED_JSK_NODELET_XXX(); \
  NODELET_DEBUG_STREAM_THROTTLE(rate, "[" << jsk_topic_tools::getFunctionName(__PRETTY_FUNCTION__).c_str() << "] " << __VA_ARGS__)
#define JSK_NODELET_INFO_STREAM_THROTTLE(rate,...) \
  JSK_TOPIC_TOOLS_DEPRECATED_JSK_NODELET_XXX(); \
  NODELET_INFO_STREAM_THROTTLE(rate, "[" << jsk_topic_tools::getFunctionName(__PRETTY_FUNCTION__).c_str() << "] " << __VA_ARGS__)
#define JSK_NODELET_WARN_STREAM_THROTTLE(rate,...) \
  JSK_TOPIC_TOOLS_DEPRECATED_JSK_NODELET_XXX(); \
  NODELET_WARN_STREAM_THROTTLE(rate, "[" << jsk_topic_tools::getFunctionName(__PRETTY_FUNCTION__).c_str() << "] " << __VA_ARGS__)
#define JSK_NODELET_ERROR_STREAM_THROTTLE(rate,...) \
  JSK_TOPIC_TOOLS_DEPRECATED_JSK_NODELET_XXX(); \
  NODELET_ERROR_STREAM_THROTTLE(rate, "[" << jsk_topic_tools::getFunctionName(__PRETTY_FUNCTION__).c_str() << "] " << __VA_ARGS__)
#define JSK_NODELET_FATAL_STREAM_THROTTLE(rate,...) \
  JSK_TOPIC_TOOLS_DEPRECATED_JSK_NODELET_XXX(); \
  NODELET_FATAL_STREAM_THROTTLE(rate, "[" << jsk_topic_tools::getFunctionName(__PRETTY_FUNCTION__).c_str() << "] " << __VA_ARGS__)

#define JSK_ROS_DEBUG(str,...) \
  JSK_TOPIC_TOOLS_DEPRECATED_JSK_ROS_XXX(); \
  ROS_DEBUG("[%s] " str, jsk_topic_tools::getFunctionName(__PRETTY_FUNCTION__).c_str(), ##__VA_ARGS__)
#define JSK_ROS_INFO(str,...) \
  JSK_TOPIC_TOOLS_DEPRECATED_JSK_ROS_XXX(); \
  ROS_INFO("[%s] " str, jsk_topic_tools::getFunctionName(__PRETTY_FUNCTION__).c_str(), ##__VA_ARGS__)
#define JSK_ROS_WARN(str,...) \
  JSK_TOPIC_TOOLS_DEPRECATED_JSK_ROS_XXX(); \
  ROS_WARN("[%s] " str, jsk_topic_tools::getFunctionName(__PRETTY_FUNCTION__).c_str(), ##__VA_ARGS__)
#define JSK_ROS_ERROR(str,...) \
  JSK_TOPIC_TOOLS_DEPRECATED_JSK_ROS_XXX(); \
  ROS_ERROR("[%s] " str, jsk_topic_tools::getFunctionName(__PRETTY_FUNCTION__).c_str(), ##__VA_ARGS__)
#define JSK_ROS_FATAL(str,...) \
  JSK_TOPIC_TOOLS_DEPRECATED_JSK_ROS_XXX(); \
  ROS_FATAL("[%s] " str, jsk_topic_tools::getFunctionName(__PRETTY_FUNCTION__).c_str(), ##__VA_ARGS__)

#define JSK_ROS_DEBUG_STREAM(...) \
  JSK_TOPIC_TOOLS_DEPRECATED_JSK_ROS_XXX(); \
  ROS_DEBUG_STREAM("[" << jsk_topic_tools::getFunctionName(__PRETTY_FUNCTION__).c_str() << "] " << __VA_ARGS__)
#define JSK_ROS_INFO_STREAM(...) \
  JSK_TOPIC_TOOLS_DEPRECATED_JSK_ROS_XXX(); \
  ROS_INFO_STREAM("[" << jsk_topic_tools::getFunctionName(__PRETTY_FUNCTION__).c_str() << "] " << __VA_ARGS__)
#define JSK_ROS_WARN_STREAM(...) \
  JSK_TOPIC_TOOLS_DEPRECATED_JSK_ROS_XXX(); \
  ROS_WARN_STREAM("[" << jsk_topic_tools::getFunctionName(__PRETTY_FUNCTION__).c_str() << "] " << __VA_ARGS__)
#define JSK_ROS_ERROR_STREAM(...) \
  JSK_TOPIC_TOOLS_DEPRECATED_JSK_ROS_XXX(); \
  ROS_ERROR_STREAM("[" << jsk_topic_tools::getFunctionName(__PRETTY_FUNCTION__).c_str() << "] " << __VA_ARGS__)
#define JSK_ROS_FATAL_STREAM(...) \
  JSK_TOPIC_TOOLS_DEPRECATED_JSK_ROS_XXX(); \
  ROS_FATAL_STREAM("[" << jsk_topic_tools::getFunctionName(__PRETTY_FUNCTION__).c_str() << "] " << __VA_ARGS__)

#define JSK_ROS_DEBUG_THROTTLE(rate, str, ...) \
  JSK_TOPIC_TOOLS_DEPRECATED_JSK_ROS_XXX(); \
  ROS_DEBUG_THROTTLE(rate, "[%s] " str, jsk_topic_tools::getFunctionName(__PRETTY_FUNCTION__).c_str(), ##__VA_ARGS__)
#define JSK_ROS_INFO_THROTTLE(rate, str, ...) \
  JSK_TOPIC_TOOLS_DEPRECATED_JSK_ROS_XXX(); \
  ROS_INFO_THROTTLE(rate, "[%s] " str, jsk_topic_tools::getFunctionName(__PRETTY_FUNCTION__).c_str(), ##__VA_ARGS__)
#define JSK_ROS_WARN_THROTTLE(rate, str, ...) \
  JSK_TOPIC_TOOLS_DEPRECATED_JSK_ROS_XXX(); \
  ROS_WARN_THROTTLE(rate, "[%s] " str, jsk_topic_tools::getFunctionName(__PRETTY_FUNCTION__).c_str(), ##__VA_ARGS__)
#define JSK_ROS_ERROR_THROTTLE(rate, str, ...) \
  JSK_TOPIC_TOOLS_DEPRECATED_JSK_ROS_XXX(); \
  ROS_ERROR_THROTTLE(rate, "[%s] " str, jsk_topic_tools::getFunctionName(__PRETTY_FUNCTION__).c_str(), ##__VA_ARGS__)
#define JSK_ROS_FATAL_THROTTLE(rate, str, ...) \
  JSK_TOPIC_TOOLS_DEPRECATED_JSK_ROS_XXX(); \
  ROS_FATAL_THROTTLE(rate, "[%s] " str, jsk_topic_tools::getFunctionName(__PRETTY_FUNCTION__).c_str(), ##__VA_ARGS__)

#define JSK_ROS_DEBUG_STREAM_THROTTLE(rate,...) \
  JSK_TOPIC_TOOLS_DEPRECATED_JSK_ROS_XXX(); \
  ROS_DEBUG_STREAM_THROTTLE(rate, "[" << jsk_topic_tools::getFunctionName(__PRETTY_FUNCTION__).c_str() << "] " << __VA_ARGS__)
#define JSK_ROS_INFO_STREAM_THROTTLE(rate,...) \
  JSK_TOPIC_TOOLS_DEPRECATED_JSK_ROS_XXX(); \
  ROS_INFO_STREAM_THROTTLE(rate, "[" << jsk_topic_tools::getFunctionName(__PRETTY_FUNCTION__).c_str() << "] " << __VA_ARGS__)
#define JSK_ROS_WARN_STREAM_THROTTLE(rate,...) \
  JSK_TOPIC_TOOLS_DEPRECATED_JSK_ROS_XXX(); \
  ROS_WARN_STREAM_THROTTLE(rate, "[" << jsk_topic_tools::getFunctionName(__PRETTY_FUNCTION__).c_str() << "] " << __VA_ARGS__)
#define JSK_ROS_ERROR_STREAM_THROTTLE(rate,...) \
  JSK_TOPIC_TOOLS_DEPRECATED_JSK_ROS_XXX(); \
  ROS_ERROR_STREAM_THROTTLE(rate, "[" << jsk_topic_tools::getFunctionName(__PRETTY_FUNCTION__).c_str() << "] " << __VA_ARGS__)
#define JSK_ROS_FATAL_STREAM_THROTTLE(rate,...) \
  JSK_TOPIC_TOOLS_DEPRECATED_JSK_ROS_XXX(); \
  ROS_FATAL_STREAM_THROTTLE(rate, "[" << jsk_topic_tools::getFunctionName(__PRETTY_FUNCTION__).c_str() << "] " << __VA_ARGS__)

#endif
