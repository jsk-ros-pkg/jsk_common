// vim: set tabstop=4 shiftwidth=4:
// Copyright (C) 2008-2009 Rosen Diankov
// 
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//     http://www.apache.org/licenses/LICENSE-2.0
// 
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <ros/node_handle.h>
#include <ros/master.h>
#include <sensor_msgs/Image.h>
#include <posedetection_msgs/ImageFeature0D.h>

#include <opencv2/highgui/highgui.hpp>
#include <boost/shared_ptr.hpp>

#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>


class Feature0DToImage
{
    ros::NodeHandle _node;
    ros::Publisher _pub;
    ros::Subscriber _sub_imagefeature;
    typedef message_filters::sync_policies::ExactTime<
    sensor_msgs::Image,
    posedetection_msgs::Feature0D
    > SyncPolicy;
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> > _sync;
    message_filters::Subscriber<sensor_msgs::Image> _sub_image;
    message_filters::Subscriber<posedetection_msgs::Feature0D> _sub_feature;
public:
    Feature0DToImage()
    { 
        ros::NodeHandle local_nh("~");

        _pub = _node.advertise<sensor_msgs::Image>(local_nh.resolveName("output"), 1);
        _sub_image.subscribe(local_nh, "image", 1);
        _sub_feature.subscribe(local_nh, "Feature0D", 1);
        _sync = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
        _sync->connectInput(_sub_image, _sub_feature);
        _sync->registerCallback(boost::bind(&Feature0DToImage::imagefeature_cb, this, _1, _2));
        _sub_imagefeature = _node.subscribe("ImageFeature0D", 1, &Feature0DToImage::imagefeature_cb, this);
    }
    virtual ~Feature0DToImage() {}

    cv::Mat draw(cv::Mat image,
                 const std::vector<float> descriptors,
                 const std::vector<float> positions,
                 const std::vector<float> scales,
                 const std::vector<float> orientations)
    {
        for(size_t i = 0; i < positions.size()/2; ++i) {
            float scale = i < scales.size() ? scales[i] : 10.0;
            cv::Point center = cv::Point(positions[2*i+0], positions[2*i+1]);
            cv::circle(image, center, scale, CV_RGB(0,255,0));
            if( i < orientations.size() ) {
                // draw line indicating orientation
                cv::Point end_pt = cv::Point(center.x+std::cos(orientations[i])*scale,
                                                center.y+std::sin(orientations[i])*scale);
                cv::line(image, center, end_pt, CV_RGB(255,0,0));
            }
        }
        return image;
    }

    void imagefeature_cb(const sensor_msgs::ImageConstPtr& image_msg,
                         const posedetection_msgs::Feature0DConstPtr& feature_msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(image_msg, "bgr8");
            cv::Mat image = draw(cv_ptr->image,
                                 feature_msg->descriptors,
                                 feature_msg->positions,
                                 feature_msg->scales,
                                 feature_msg->orientations);
            _pub.publish(cv_bridge::CvImage(cv_ptr->header, "bgr8", image));
        } catch (cv_bridge::Exception& error) {
            ROS_WARN("bad frame");
            return;
        }
    }

    void imagefeature_cb(const posedetection_msgs::ImageFeature0DConstPtr& msg_ptr)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg_ptr->image, "bgr8");
            cv::Mat image = draw(cv_ptr->image,
                                 msg_ptr->features.descriptors,
                                 msg_ptr->features.positions,
                                 msg_ptr->features.scales,
                                 msg_ptr->features.orientations);
            _pub.publish(cv_bridge::CvImage(cv_ptr->header, "bgr8", image));
        } catch (cv_bridge::Exception& error) {
            ROS_WARN("bad frame");
            return;
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "feature0d_to_image");
    boost::shared_ptr<Feature0DToImage> node(new Feature0DToImage());
    ros::spin();
    return 0;
}

