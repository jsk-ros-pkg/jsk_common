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

////////////////////////////////////////////////////////
// a test code for jsk_topic_tools/rosparam_utils.{h, cpp}
////////////////////////////////////////////////////////

#include "jsk_topic_tools/rosparam_utils.h"
#include <gtest/gtest.h>

TEST(readVectorParameter, stringTest)
{
  ros::NodeHandle nh("~");
  std::vector<std::string> result;
  bool ret = jsk_topic_tools::readVectorParameter(nh, "string_vector", result);
  ASSERT_TRUE(ret);
  ASSERT_EQ(result.size(), 3);
  ASSERT_EQ(result[0], "string_value0");
  ASSERT_EQ(result[1], "string_value1");
  ASSERT_EQ(result[2], "string_value2");
}

TEST(readVectorParameter, doubleTest)
{
  ros::NodeHandle nh("~");
  std::vector<double> result;
  bool ret = jsk_topic_tools::readVectorParameter(nh, "double_vector", result);
  ASSERT_TRUE(ret);
  ASSERT_EQ(result.size(), 4);
  ASSERT_EQ(result[0], 0.0);
  ASSERT_EQ(result[1], 1.0);
  ASSERT_EQ(result[2], 2.0);
  ASSERT_EQ(result[3], 3.0);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_rosparam_utils");
  return RUN_ALL_TESTS();
}
