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

#include "BufferedNode.h"
#include "Buffer.h"
#include "Vector.h"
#include "Map.h"
#include "Source.h"
#include "Matrix.h"
#include <sstream>
#include <iomanip>

#include <sys/time.h>

#include <boost/shared_ptr.hpp>

#include <../config.h>

#ifdef ENABLE_ROS
#include <ros/ros.h>
#include "jsk_hark_msgs/HarkPower.h"
#include "HarkRosGlobals.h"
#include "TimeStamp.h"
#include <math.h>
#include <vector>

using namespace std;
using namespace FD;

class RosJskHarkMsgsPublisher;

DECLARE_NODE(RosJskHarkMsgsPublisher)
    /*Node
     *
     * @name RosJskHarkMsgsPublisher
     * @category JSK:ROS:IO
     * @description ROS msg publisher node for hark common messages (jsk_hark_msgs).
     *
     * @parameter_name ADVANCE
     * @parameter_type int
     * @parameter_value 160
     * @parameter_description Shift sample number for sliding spectrum analysis.
     *
     * @parameter_name ENABLE_DEBUG
     * @parameter_type bool
     * @parameter_value false
     * @parameter_list true:false
     * @parameter_description print debug message of this module in case of true.
     * 
     * @parameter_name TOPIC_NAME_HARKPOWER
     * @parameter_type string
     * @parameter_value HarkPower
     * @parameter_description Published topic name for ROS (HarkPower type message)
     * @parameter_name BUFFER_NUM
     *
     * @parameter_type int
     * @parameter_value 100
     * @parameter_description Buffer size for a ROS published message
     * 
     * @parameter_name ROS_LOOP_RATE
     * @parameter_type float
     * @parameter_value 100000
     * @parameter_description This allows you to specify a frequency that you would like to loop at [Hz]. Keep this value large. (If ROS interval is shorter than HARK interval, ROS interval is overwritten.)
     *
     * @parameter_name TIMESTAMP_TYPE
     * @parameter_type string
     * @parameter_value ROS_TIME_NOW
     * @parameter_list ROS_TIME_NOW:CONSTANT_INCREMENT
     * @parameter_description Time stamp type. If TIMESTAMP is connected, this is ignored.
     * 
     * @parameter_name SAMPLING_RATE
     * @parameter_type int
     * @parameter_value 16000
     * @parameter_valid TIMESTAMP_TYPE=CONSTANT_INCREMENT
     * @parameter_description The time increment is caluculated as ADVANCE / SAMPLING_RATE
     * 
     * @parameter_name ROS_FRAME_ID
     * @parameter_type string
     * @parameter_value HarkRosFrameID
     * @parameter_description ROS frame_id of the message header
     *
     * @input_name SRC_POWER
     * @input_type Vector<float>
     * @input_description all signal Power spectrum after sound source location.
     *
     * @input_name TIMESTAMP
     * @input_type TimeStamp
     * @input_description TimeStamp for published messages. This is optional input. If not connected, ros::Time::now() is stamped in the messages.
     *
     * @output_name OUTPUT
     * @output_type ObjectRef
     * @output_description This is a dummy output, and it has no mean. Only for an activation of this module.
     *
     *
     END*/

    class RosJskHarkMsgsPublisher : public BufferedNode {
        bool enable_debug;     // flag whether print debug message or not.
        int advance;          // shift length of STFT.

        int src_powerID;
        int TimeStampID;
        int outputID;

        // ROS related parameters
        string topic_name_harkpower;
        int buffer_num;
        float ros_loop_rate;
        string ros_frame_id;
        string timestamp_type;
        int sampling_rate;
        float const_in_sec;
        ros::Time oHarkTimeBase;

        private:
        ros::Publisher _pub_harkpower;

        public:
        RosJskHarkMsgsPublisher(string nodeName, ParameterSet params)
            : BufferedNode(nodeName, params),
            src_powerID(-1),
            TimeStampID(-1)
        {

            advance = dereference_cast<int>(parameters.get("ADVANCE"));
            enable_debug = dereference_cast<bool>(parameters.get("ENABLE_DEBUG"));
            topic_name_harkpower          = object_cast<String>(parameters.get("TOPIC_NAME_HARKPOWER"));
            buffer_num  = dereference_cast<int>(parameters.get("BUFFER_NUM"));
            ros_loop_rate  = dereference_cast<float>(parameters.get("ROS_LOOP_RATE"));
            ros_frame_id = object_cast<String>(parameters.get("ROS_FRAME_ID"));
            timestamp_type = object_cast<String>(parameters.get("TIMESTAMP_TYPE"));
            sampling_rate = dereference_cast<int>(parameters.get("SAMPLING_RATE"));
            const_in_sec = (float)advance / (float)sampling_rate;

            outputID = addOutput("OUTPUT");

            inOrder = true;
            cout << getName() << " constructor end..." << endl;

        }

        virtual void initialize()
        {
            cout << getName() << " initialized..." << endl;
            if (src_powerID != -1){
                _pub_harkpower         = __MasterRosNodeHandlerForJsk__->advertise<jsk_hark_msgs::HarkPower>(topic_name_harkpower, buffer_num, false);
            }
            this->BufferedNode::initialize();
        }


        // dynamic input-port translation
        virtual int translateInput(string inputName) {
            //if (inputName == "STRING") {
            //    return strID = addInput(inputName);
            //}
            //else
            // check whether the input ports have arc from other module.
            if (inputName == "SRC_POWER") {
                return src_powerID = addInput(inputName);
            }
            else if (inputName == "TIMESTAMP") {
                return TimeStampID = addInput(inputName);
            }
            else {
                throw new NodeException(this, inputName
                        + " is not supported.", __FILE__, __LINE__);
            }
        }

        // process per one iteration
        void calculate(int output_id, int count, Buffer &out) {

            int bytes;    
            RCPtr<Vector<float> > src_power_ptr;

            // bind objects of input-port to local variable
            if (src_powerID != -1){
                src_power_ptr = getInput(src_powerID, count);
                out[count] = src_power_ptr;
            }

            jsk_hark_msgs::HarkPower harkpowermsg;

            harkpowermsg.count = count;

            if(TimeStampID == -1){
                if (timestamp_type == "ROS_TIME_NOW") {      
                    ros::Time oHarkTime = ros::Time::now();
                    harkpowermsg.header.stamp = oHarkTime;
                } else if (timestamp_type == "CONSTANT_INCREMENT"){
                    if(count == 0) oHarkTimeBase = ros::Time::now();
                    ros::Time oHarkTime = oHarkTimeBase + ros::Duration(const_in_sec * (float)count);
                    harkpowermsg.header.stamp = oHarkTime;
                }      
            }else{
                ObjectRef oBaseTimeStamp = getInput(TimeStampID,count);
                const TimeStamp& BaseTime = object_cast<TimeStamp> (oBaseTimeStamp);
                harkpowermsg.header.stamp.sec         = BaseTime.time.tv_sec;
                harkpowermsg.header.stamp.nsec         = BaseTime.time.tv_usec * 1000;
            }

            harkpowermsg.header.frame_id = ros_frame_id;

            ros::Rate loop_rate(ros_loop_rate);

            ///////////////////////////////////////////////////////////////
            //
            //      bodies
            //
            ///////////////////////////////////////////////////////////////

            //----- for source array Power spectrum signals
            //////////////////////////////////////////////////////////////
            if (!src_power_ptr.isNil()) {

                int directions = (*src_power_ptr).size();

                if (enable_debug == true) printf("SRC_POWER: %d\n", directions);
                bytes = directions * sizeof(float);

                harkpowermsg.directions = directions;
                harkpowermsg.data_bytes = bytes;
                harkpowermsg.powers.resize(directions);
                for (int c = 0; c < directions; c++) {
                    harkpowermsg.powers[c] = (float)(*src_power_ptr)[c];
                }
                bool _allzero = true;
                for (int c = 0; c < directions; c++) {
                    if ((float)(*src_power_ptr)[c] != 0.0) {
                        _allzero = false;
                        break;
                    }
                }
                if (!_allzero) {
                    _pub_harkpower.publish(harkpowermsg);
                }
            }

            loop_rate.sleep();

        }

        IN_ORDER_NODE_SPEEDUP(RosJskHarkMsgsPublisher)

    };

#endif
