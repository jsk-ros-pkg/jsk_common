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

#include <string>
#include <vector>
#include "jsk_topic_tools/log_utils.h"


namespace jsk_topic_tools
{

  bool warnNoRemap(const std::vector<std::string> names)
  {
    bool no_warning = true;
    ros::M_string remappings = ros::names::getRemappings();
    for (size_t i = 0; i < names.size(); i++)
    {
      std::string resolved_name = ros::names::resolve(/*name=*/names[i],
                                                      /*_remap=*/false);
      if (remappings.find(resolved_name) == remappings.end())
      {
        ROS_WARN("[%s] '%s' has not been remapped.",
                 ros::this_node::getName().c_str(), names[i].c_str());
        no_warning = false;
      }
    }
    return no_warning;
  }

  const std::string getFunctionName(const std::string &name)
  {
      size_t end = name.rfind('(');
      if(end == std::string::npos)
      {
          end = name.size();
      }
      size_t begin = 1 + name.rfind(' ', end);
      return name.substr(begin, end - begin);
  }

}  // namespace jsk_topic_tools
