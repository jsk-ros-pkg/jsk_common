/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2010, Kyoto University and Honda Motor Co.,Ltd. All rights reserved.
 *
 * HARK was developed by researchers in Okuno Laboratory at the Kyoto University and 
 * Honda Research Institute Japan Co.,Ltd.
 *
 * Redistribution and use in source and binary forms, with or without modification, are 
 * permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this list 
 *    of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice, this 
 *    list of conditions and the following disclaimer in the documentation and/or other 
 *    materials provided with the distribution.
 *  * Neither the name of Kyoto University and Honda Motor Co.,Ltd., Inc. nor the names 
 *    of its contributors may be used to endorse or promote products derived from this 
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY 
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES 
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT 
 * SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCI-
 * DENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSI-
 * NESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTR-
 * ACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY 
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <iostream>
#include "Node.h"
#include <../config.h>

#ifdef ENABLE_ROS

#include <ros/ros.h>

using namespace std;
using namespace FD;

class RosNodeGeneratorForJSK;

DECLARE_NODE(RosNodeGeneratorForJSK)
/*Node
 *
 * @name RosNodeGeneratorForJSK
 * @category JSK:ROS:Master
 * @description Generate a main ROS node
 *
 * @output_name VALUE
 * @output_type int
 * @output_description int parameter
 * 
 * @parameter_name NODE_NAME
 * @parameter_type string
 * @parameter_value HARK_MASTER_NODE
 * @parameter_description Node name for ROS
 *
END*/

ros::NodeHandle *__MasterRosNodeHandlerForJsk__;

class RosNodeGeneratorForJSK : public Node{
  int outputID;
  string node_name;

public:
  RosNodeGeneratorForJSK(string nodeName, ParameterSet params)
    : Node(nodeName, params) 
  {
    outputID = addOutput("VALUE");
    node_name = object_cast<String>(parameters.get("NODE_NAME"));

    std::vector<std::pair<std::string, std::string> > args;
    //ros::init(args, node_name, ros::init_options::NoSigintHandler);
    __MasterRosNodeHandlerForJsk__ = new ros::NodeHandle;

    cout << "ROS node : " << node_name << " generated..." << endl;
  }

  ~RosNodeGeneratorForJSK()
  {
    cout << "ROS destructed..." << endl;    
  }
  
  void initialize()
  {    
    cout << "ROS initialized..." << endl;    
    this->Node::initialize();
  }
  
  void reset()
  {
    cout << "ROS reset..." << endl;    
    this->Node::reset();
  }
  
  ObjectRef getOutput(int output_id, int count)
  {
    if (output_id==outputID) return ObjectRef(Int::alloc(0));
    else throw new NodeException (this, "Unknown output id", __FILE__, __LINE__);
  }

};

#endif
