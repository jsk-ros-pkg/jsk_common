#include <cstdio>
#include <vector>
#include <list>
#include <boost/lambda/lambda.hpp>
#include "ros/ros.h"
#include "ros/header.h"
#include "ros/console.h"
#include "std_msgs/Header.h"
#include "jsk_topic_tools/List.h"
#include "jsk_topic_tools/Update.h"
#include "topic_tools/shape_shifter.h"
#include "topic_tools/parse.h"

using std::string;
using std::vector;
using std::list;
using namespace topic_tools;

class pub_info_t
{
public:
    std::string topic_name;
    ros::Publisher pub;
    ros::Subscriber *sub;
    bool advertised;
    boost::shared_ptr<ShapeShifter > msg;
    ros::Timer timer;
    ros::Duration rate;
    ros::Time last_time_received;
    bool topic_with_header;
    bool latched;
    uint32_t last_seq_received;
    uint32_t topic_received;

    void publish(const ros::TimerEvent &event)
    {
        if ( advertised == false ) return;

        topic_received++;
        if ( topic_with_header == true ) {
            std_msgs::Header header;
            uint8_t buf[msg->size()];
            ros::serialization::OStream ostream(buf, msg->size());
            ros::serialization::IStream istream(buf, msg->size());
            msg->write(ostream);
            ((uint32_t *)buf)[0] = last_seq_received + topic_received;
            ros::Time tmp(last_time_received.toSec() + topic_received * rate.toSec());
            ((uint32_t *)buf)[1] = tmp.sec;
            ((uint32_t *)buf)[2] = tmp.nsec;
            msg->read(istream);
        }
        pub.publish(msg);
    }
};

typedef boost::shared_ptr<pub_info_t> pub_info_ref;

static list<pub_info_ref> g_pubs;

static ros::NodeHandle *g_node = NULL;

static bool fixed_rate;

void in_cb(const boost::shared_ptr<ShapeShifter const>& msg,
           boost::shared_ptr<pub_info_t> s)
{
    using namespace boost::lambda;

    s->msg = boost::const_pointer_cast<ShapeShifter>(msg);
    s->topic_received = 0; // reset topic_received
    if ( s->advertised == false ) {
        s->pub = msg->advertise(*g_node, s->topic_name+string("_buffered"), 10, s->latched);
        s->advertised = true;
    }
    ROS_INFO_STREAM("advertised as " << s->topic_name+string("_buffered") << " running at " << 1/(s->rate.toSec()) << "Hz");

    // check if msg has header
    {
        std_msgs::Header header;
        uint8_t buf[msg->size()];
        ros::serialization::OStream stream(buf, msg->size());
        msg->write(stream);
        header.seq = ((uint32_t *)buf)[0];
        header.stamp.sec = ((uint32_t *)buf)[1];
        header.stamp.nsec = ((uint32_t *)buf)[2];

        if ( abs((header.stamp - ros::Time::now()).toSec()) < 5.0 ) {
            ROS_INFO_STREAM(" this message contains headers.. seq =" <<  header.seq << " stamp = " << header.stamp);
            s->topic_with_header = true;
            s->last_seq_received = header.seq;
            s->last_time_received = header.stamp;
        }
    }
    //g_node->createTimer(ros::Duration(0.1), [](const ros::TimerEvent event) { std::cerr << "hoge" << std::endl; });
    {// at first publish once
      ros::TimerEvent ev;
      s->publish (ev);
    }
    s->timer = g_node->createTimer(s->rate, &pub_info_t::publish, s);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "topic_buffer_client", ros::init_options::AnonymousName);

    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    double rate = 0.1; // 10Hz
    if (nh.hasParam("fixed_rate")) {
      fixed_rate = true;
      nh.param ("fixed_rate", rate, 0.1);
      ROS_INFO("use fixed rate = %f", rate);
    }

    double update_rate = 10; // 0.1Hz
    if (nh.hasParam("update_rate")) {
      nh.param ("update_rate", update_rate, 10.0);
      ROS_INFO("use update rate = %f", update_rate);
    }

    bool latched;
    if (nh.hasParam("latched")) {
      nh.param ("latched", latched, false);
      if(latched) {
        ROS_INFO("use latched");
      }
    }

    g_node = &n;

    // New service
    ros::service::waitForService(string("/list"), -1);
    ros::ServiceClient sc_list = n.serviceClient<jsk_topic_tools::List>(string("/list"), true);

    jsk_topic_tools::List::Request req;
    jsk_topic_tools::List::Response res;

    ROS_INFO_STREAM("calling /list");
    while ( sc_list.call(req, res) == false) {
        ROS_WARN_STREAM("calling /list fails, retry...");
        ros::Duration(1).sleep();
    }
    ROS_WARN_STREAM("calling /list success!!!");
    for(vector<jsk_topic_tools::TopicInfo>::iterator it = res.info.begin(); it != res.info.end(); ++it) {
        boost::shared_ptr<pub_info_t> pub_info(new pub_info_t);
        pub_info->topic_name = it->topic_name;
        if (fixed_rate) {
          pub_info->rate = ros::Duration(rate);
        } else {
          pub_info->rate = ros::Duration(it->rate);
        }
        pub_info->latched = latched;
        pub_info->advertised = false;
        pub_info->topic_with_header = false;
        ROS_INFO_STREAM("subscribe " << pub_info->topic_name+string("_update") << " at " << pub_info->rate);
        pub_info->sub = new ros::Subscriber(n.subscribe<ShapeShifter>(pub_info->topic_name+string("_update"), 10, boost::bind(in_cb, _1, pub_info)));

        g_pubs.push_back(pub_info);
    }
    ROS_INFO_STREAM("calling /list has done.. found " << res.info.size() << " topics to publish");

    ros::Rate rate_loop(100);
    ros::Time last_updated;

    ros::ServiceClient sc_update = n.serviceClient<jsk_topic_tools::Update>(string("/update"), true);
    while ( ros::ok() ) {

        if ( ! fixed_rate && (ros::Time::now() - last_updated > ros::Duration(update_rate)) ) {
            for (list<pub_info_ref>::iterator it = g_pubs.begin();
                 it != g_pubs.end();
                 ++it) {
                jsk_topic_tools::Update::Request req;
                jsk_topic_tools::Update::Response res;
                req.topic = (*it)->topic_name;
                if ( sc_update.call(req, res) == false ) {
                    ROS_ERROR_STREAM("calling /update (" << req.topic << ") fails, retry...");
                    continue;
                }
                (*it)->rate = ros::Duration(res.rate);
                ROS_INFO_STREAM("calling /update " << req.topic << " .. " << res.rate);
            }
            last_updated = ros::Time::now();
        }

        ros::spinOnce();
        rate_loop.sleep();
    }

    return 0;
}
