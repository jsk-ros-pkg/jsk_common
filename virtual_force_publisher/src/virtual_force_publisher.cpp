/*********************************************************************
 * Software License Agreement (BSD License)
 * 
 *  Copyright (c) 2012, JSK, The University of Tokyo.
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

/* Author: Kei Okada */

#include <boost/scoped_ptr.hpp>
#include <ros/ros.h>
#include <urdf/model.h>
#include <kdl/tree.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <tf_conversions/tf_kdl.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_listener.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

typedef boost::shared_ptr<sensor_msgs::JointState const> JointStateConstPtr;

namespace virtual_force_publisher{

    class VirtualForcePublisher
    {
    private:
        KDL::JntArray jnt_pos_;
        KDL::Jacobian jacobian_;
        KDL::Chain chain_;
        boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;
        ros::Subscriber joint_state_sub_;
        ros::Publisher  wrench_stamped_pub_;
        std::string root, tip;

        ros::Duration publish_interval_;
        std::map<std::string, ros::Time> last_publish_time_;
        double t_const_;
        KDL::Wrench F_pre_;
        tf::TransformListener listener_;
    public:

        VirtualForcePublisher()
        {
            ros::NodeHandle n_tilde("~");
            ros::NodeHandle n;

            // subscribe to joint state
            joint_state_sub_ = n.subscribe("joint_states", 1, &VirtualForcePublisher::callbackJointState, this);
            wrench_stamped_pub_ = n.advertise<geometry_msgs::WrenchStamped>("wrench", 1);

            // set publish frequency
            double publish_freq;
            n_tilde.param("publish_frequency", publish_freq, 50.0);
            publish_interval_ = ros::Duration(1.0/std::max(publish_freq,1.0));

	    //set time constant of low pass filter
	    n_tilde.param("time_constant", t_const_, 0.3);
            // set root and tip
            n_tilde.param("root", root, std::string("torso_lift_link"));
            n_tilde.param("tip", tip, std::string("l_gripper_tool_frame"));
	    
            // load robot model
            urdf::Model robot_model;
            robot_model.initParam("robot_description");

            KDL::Tree kdl_tree;
            kdl_parser::treeFromUrdfModel(robot_model, kdl_tree);
            kdl_tree.getChain(root, tip, chain_);
            jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(chain_));

            jnt_pos_.resize(chain_.getNrOfJoints());
            jacobian_.resize(chain_.getNrOfJoints());

            for (size_t i=0; i<chain_.getNrOfSegments(); i++){
                if (chain_.getSegment(i).getJoint().getType() != KDL::Joint::None){
                    std::cerr << "kdl_chain(" << i << ") " << chain_.getSegment(i).getJoint().getName().c_str() << std::endl;
                }
            }
        }
        ~VirtualForcePublisher() { }

        void callbackJointState(const JointStateConstPtr& state)
        {
            std::map<std::string, double> joint_name_position;
            if (state->name.size() != state->position.size()){
                ROS_ERROR("Robot state publisher received an invalid joint state vector");
                return;
            }

            // determine least recently published joint
            ros::Time last_published = ros::Time::now();
            for (unsigned int i=0; i<state->name.size(); i++) {
                ros::Time t = last_publish_time_[state->name[i]];
                last_published = (t < last_published) ? t : last_published;
            }

            if (state->header.stamp >= last_published + publish_interval_){
                // get joint positions from state message
                std::map<std::string, double> joint_positions;
                std::map<std::string, double> joint_efforts;
                for (unsigned int i=0; i<state->name.size(); i++) {
                    joint_positions.insert(make_pair(state->name[i], state->position[i]));
                    joint_efforts.insert(make_pair(state->name[i], state->effort[i]));
                }

                KDL::JntArray  tau;
                KDL::Wrench    F;
		Eigen::Matrix<double,Eigen::Dynamic,6> jac_t;
		Eigen::Matrix<double,6,Eigen::Dynamic> jac_t_pseudo_inv;
		tf::StampedTransform transform;
		KDL::Wrench    F_pub;
		tf::Vector3 tf_force;
		tf::Vector3 tf_torque;

                tau.resize(chain_.getNrOfJoints());

                //getPositions;
                for (size_t i=0, j=0; i<chain_.getNrOfSegments(); i++){
                    if (chain_.getSegment(i).getJoint().getType() == KDL::Joint::None)
                        continue;

                    // check
                    std::string joint_name = chain_.getSegment(i).getJoint().getName();
                    if ( joint_positions.find(joint_name) == joint_positions.end() ) {
                        ROS_ERROR("Joint '%s' is not found in joint state vector", joint_name.c_str());
                    }

                    // set position
                    jnt_pos_(j) = joint_positions[joint_name];
                    tau(j) = joint_efforts[joint_name];
                    j++;
                }
		
		jnt_to_jac_solver_->JntToJac(jnt_pos_, jacobian_);
		jac_t = jacobian_.data.transpose();
		if ( jacobian_.columns() >= jacobian_.rows() ) {
		  jac_t_pseudo_inv =(jac_t.transpose() * jac_t).inverse() *  jac_t.transpose();
		} else {
		  jac_t_pseudo_inv =jac_t.transpose() * ( jac_t *  jac_t.transpose() ).inverse();
		}
#if 1
		{
		  ROS_INFO("jac_t# jac_t : ");
		  Eigen::Matrix<double,6,6> mat_i =  mat_i = jac_t_pseudo_inv * jac_t;
		  for (unsigned int i = 0; i < 6; i++) {
		    std::stringstream ss;
		    for (unsigned int j=0; j<6; j++)
		      ss << std::fixed << std::setw(8) << std::setprecision(4) << mat_i(j,i) << " ";
		    ROS_INFO_STREAM(ss.str());
                    }
		}
#endif
                // f = - inv(jt) * effort
                for (unsigned int j=0; j<6; j++)
                    {
                        F(j) = 0;
                        for (unsigned int i = 0; i < jnt_pos_.rows(); i++)
                            {
                                F(j) += -1 * jac_t_pseudo_inv(j, i) * tau(i);
                            }
                    }

		//low pass filter
		F += (F_pre_ - F) * exp(-1.0 / t_const_);
		for (unsigned int j=0; j<6; j++){
		  F_pre_(j) = 0;
		}
		F_pre_ += F;

		//tf transformation
		tf::vectorKDLToTF(F.force, tf_force);
		tf::vectorKDLToTF(F.torque, tf_torque);
		try{
		  listener_.waitForTransform( tip, root, state->header.stamp, ros::Duration(1.0));
		  listener_.lookupTransform( tip, root, state->header.stamp , transform);
		}   
		catch (tf::TransformException ex){
		  ROS_ERROR("%s",ex.what());
		  ros::Duration(1.0).sleep();
		}
		for (unsigned int j=0; j<3; j++){
		  F_pub.force[j] = transform.getBasis()[j].dot(tf_force);
		  F_pub.torque[j] = transform.getBasis()[j].dot(tf_torque);
		}

                geometry_msgs::WrenchStamped msg;
                msg.header.stamp = state->header.stamp;
                msg.header.frame_id = tip;
                // http://code.ros.org/lurker/message/20110107.151127.7177af06.nl.html
                //tf::WrenchKDLToMsg(F,msg.wrench);
                msg.wrench.force.x = F_pub.force[0];
                msg.wrench.force.y = F_pub.force[1];
                msg.wrench.force.z = F_pub.force[2];
                msg.wrench.torque.x = F_pub.torque[0];
                msg.wrench.torque.y = F_pub.torque[1];
                msg.wrench.torque.z = F_pub.torque[2];
                wrench_stamped_pub_.publish(msg);

                {
                    ROS_INFO("jacobian : ");
                    for (unsigned int i = 0; i < jnt_pos_.rows(); i++) {
                        std::stringstream ss;
                        for (unsigned int j=0; j<6; j++)
                            ss << std::fixed << std::setw(8) << std::setprecision(4) << jacobian_(j,i) << " ";
                        ROS_INFO_STREAM(ss.str());
                    }
                    ROS_INFO("effort : ");
                    std::stringstream sstau;
                    for (unsigned int i = 0; i < tau.rows(); i++) {
                        sstau << std::fixed << std::setw(8) << std::setprecision(4) << tau(i) << " ";
                    }
                    ROS_INFO_STREAM(sstau.str());
                    ROS_INFO("force : ");
                    std::stringstream ssf;
                    for (unsigned int j = 0; j < 6; j++) {
		      ssf << std::fixed << std::setw(8) << std::setprecision(4) << F(j) << " ";
                    }
                    ROS_INFO_STREAM(ssf.str());
                }

                // store publish time in joint map
                for (unsigned int i=0; i<state->name.size(); i++)
                    last_publish_time_[state->name[i]] = state->header.stamp;
            }
        }
    };
};

using namespace virtual_force_publisher;

int main (int argc, char ** argv) {

    // Initialize ros
    ros::init(argc, argv, "virtual_force_publisher");

    VirtualForcePublisher virtual_force_publisher;

    ros::spin();

    return 0;
}
