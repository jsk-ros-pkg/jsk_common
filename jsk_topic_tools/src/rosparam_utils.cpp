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
 *   * Neither the name of the Willow Garage nor the names of its
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

#include "jsk_topic_tools/rosparam_utils.h"

namespace jsk_topic_tools
{
  double getXMLDoubleValue(XmlRpc::XmlRpcValue val)
  {
    switch(val.getType())
    {
    case XmlRpc::XmlRpcValue::TypeInt:
    {
      return (double)((int)val);
    }
    case XmlRpc::XmlRpcValue::TypeDouble:
    {
      return (double)val;
    }
    default:
    {
      throw std::runtime_error("the value cannot be converted into double");
    }
    }
  }

  bool readVectorParameter(ros::NodeHandle& nh,
                           const std::string& param_name,
                           std::vector<double>& result)
  {
    if (nh.hasParam(param_name)) {
      XmlRpc::XmlRpcValue v;
      nh.param(param_name, v, v);
      if (v.getType() == XmlRpc::XmlRpcValue::TypeArray) {
        result.resize(v.size());
        for (size_t i = 0; i < result.size(); i++) {
          result[i] = getXMLDoubleValue(v[i]);
        }
        return true;
      }
      else {
        return false;
      }
    }
    else {
      return false;
    }
  }

  bool readVectorParameter(ros::NodeHandle& nh,
                           const std::string& param_name,
                           std::vector<std::string>& result)
  {
    if (nh.hasParam(param_name)) {
      XmlRpc::XmlRpcValue v;
      nh.param(param_name, v, v);
      if (v.getType() == XmlRpc::XmlRpcValue::TypeArray) {
        result.resize(v.size());
        for (size_t i = 0; i < result.size(); i++) {
          if (v[i].getType() == XmlRpc::XmlRpcValue::TypeString) {
            result[i] = (std::string)v[i];
          }
          else {
            throw std::runtime_error(
              "the value cannot be converted into std::string");
          }
        }
        return true;
      }
      else {
        return false;
      }
    }
    else {
      return false;
    }
  }

  
}
