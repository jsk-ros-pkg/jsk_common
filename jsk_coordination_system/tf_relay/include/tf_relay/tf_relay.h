#ifndef TF_RELAY_TF_RELAY_H__
#define TF_RELAY_TF_RELAY_H__

// Standaerd C++ Library
#include <iostream>
// ROS
#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
// ROS message
#include <geometry_msgs/TransformStamped.h>
// Boost
#include <boost/bind.hpp>

namespace tf_relay {

    /**
     * @brief The TFRelay class is for calculate and publish walking status of each foot
     */
    class TFRelay
    {
        public:

            TFRelay( ros::NodeHandle &nh,
                     ros::NodeHandle &nh_private,
                     tf2_ros::Buffer &tf_buffer,
                     tf2_ros::TransformBroadcaster &tf_broadcaster );

            /**
             * @brief Main loop function. It will start spinner threads for subscribers and start main loop
             * for calculation of commands to vibrators.
             * @param nh ros::NodeHandle ros node handler
             * @param nh_private ros::NodeHandle ros node handler with private namespace
             * @param tf_buffer tf2_ros::Buffer tf2_ros buffer
             */
            void spin();

        private:

            /**
             * ROS
             */
            ros::NodeHandle& nh_;
            ros::NodeHandle& nh_private_;
            tf2_ros::Buffer& tf_buffer_;
            tf2_ros::TransformBroadcaster& tf_broadcaster_;
            std::string reference_frame_id_;
            std::string output_frame_id_;
            std::string fixed_frame_id_;
            ros::Duration duration_timeout_;
            bool identity_initialize_;

            bool initialized_;

            geometry_msgs::TransformStamped msg_transform_;

            /**
             *
             */
            void callbackTimerTF( const ros::TimerEvent& );
    };

}

#endif
