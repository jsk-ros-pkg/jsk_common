#include <cstdio>
#include <vector>
#include <list>
#include "ros/console.h"
#include "std_msgs/String.h"
#include "jsk_topic_tools/List.h"
#include "jsk_topic_tools/Update.h"
#include "topic_tools/shape_shifter.h"
#include "topic_tools/parse.h"

using std::string;
using std::vector;
using std::list;
using namespace topic_tools;

class sub_info_t
{
public:
    std::string topic_name;
    ros::Subscriber *sub;
    ros::Publisher pub;
    bool advertised;
    boost::shared_ptr<ShapeShifter const> msg;
    ros::Time last_time_received;
    ros::Duration rate;
};

typedef boost::shared_ptr<sub_info_t> sub_info_ref;

static list<sub_info_ref> g_subs;

static ros::NodeHandle *g_node = NULL;

void in_cb(const boost::shared_ptr<ShapeShifter const>& msg,
           boost::shared_ptr<sub_info_t> s)
{
    if ( s->rate.isZero() && s->last_time_received.isZero() ) { // skip first time
    } else if ( s->rate.isZero() && ! s->last_time_received.isZero() ) { // just for second time
        s->rate = ros::Time::now() - s->last_time_received;
    } else {
        double alpha = 0.1; //N = 19 alpha =  2 / ( N + 1 )
        s->rate = (ros::Duration(alpha * (ros::Time::now() - s->last_time_received).toSec() + (1 - alpha) * s->rate.toSec()));
    }

    if ( s->advertised == false ) {
        s->pub = msg->advertise(*g_node, s->topic_name+string("_update"), 10);
        s->advertised = true;
        ROS_INFO_STREAM("advertised as " << s->topic_name+string("_update"));
    }

    s->msg = msg;

    // update last_time_received
    s->last_time_received = ros::Time::now();
}


bool list_topic_cb(jsk_topic_tools::List::Request& req,
                   jsk_topic_tools::List::Response& res)
{
    ROS_INFO_STREAM("service (list) is called, returns " << g_subs.size() << " topics");
    for (list<sub_info_ref>::iterator it = g_subs.begin();
         it != g_subs.end();
         ++it)
        {
            while( ! (*it)->advertised ) {
                ROS_WARN_STREAM("service (list) waiting.. " << (*it)->topic_name << " is not running yet...");
                return false;
            }
            res.topic_names.push_back((*it)->topic_name);
            ROS_INFO_STREAM("service (list) returns res.topic_name:" << (*it)->topic_name );
        }

    return true;
}

void update_topic_cb(const std_msgs::String::ConstPtr &msg) {
  ROS_INFO_STREAM("topic (update) is called, searching from " << g_subs.size() << " topics");
  for (list<sub_info_ref>::iterator it = g_subs.begin();
         it != g_subs.end();
         ++it)
  {
    if ( (*it)->topic_name == msg->data && (*it)->advertised == true ) {
      if (! (*it)->advertised ) {
        ROS_WARN_STREAM("topic (update) " << (*it)->topic_name << " is not running yet...");
        continue;
      }
      ROS_INFO_STREAM("topic (update) " << (*it)->topic_name << " running at " << 1.0/((*it)->rate).toSec() << " Hz");
      (*it)->pub.publish((*it)->msg);
      ROS_INFO_STREAM("topic (update) is called, req.topic:" << msg->data);
      return;
    }
  }
  ROS_ERROR_STREAM("could not find topic named " << msg->data );
}

bool update_topic_cb(jsk_topic_tools::Update::Request& req,
                     jsk_topic_tools::Update::Response& res)
{

    ROS_INFO_STREAM("service (update) is called, searching from " << g_subs.size() << " topics");
    for (list<sub_info_ref>::iterator it = g_subs.begin();
         it != g_subs.end();
         ++it)
        {
            if ( (*it)->topic_name == req.topic_name && (*it)->advertised == true ) {
                if (! (*it)->advertised ) {
                    ROS_WARN_STREAM("service (update) " << (*it)->topic_name << " is not running yet...");
                    continue;
                }
                ROS_INFO_STREAM("service (update) " << (*it)->topic_name << " running at " << 1.0/((*it)->rate).toSec() << " Hz");
                (*it)->pub.publish((*it)->msg);
                res.rate = (*it)->rate.toSec();
                ROS_INFO_STREAM("service (update) is called, req.topic:" << req.topic_name << ", res.rate " << res.rate);
                return true;
            }
        }
    ROS_ERROR_STREAM("could not find topic named " << req.topic_name );
    return false;
}


int main(int argc, char **argv)
{
    vector<string> args;
    ros::removeROSArgs(argc, (const char**)argv, args);

    if (args.size() < 2)
        {
            printf("\nusage: topic_buffer_Server IN_TOPIC1 [IN_TOPIC2 [...]]\n\n");
            return 1;
        }

    ros::init(argc, argv, "topic_buffer_server");
    vector<string> topics;
    for (unsigned int i = 1; i < args.size(); i++)
        topics.push_back(args[i]);

    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    g_node = &n;

    for (size_t i = 0; i < topics.size(); i++)
        {
            //sub_info_t sub_info;
            boost::shared_ptr<sub_info_t> sub_info(new sub_info_t);
            sub_info->topic_name = ros::names::resolve(topics[i]);
            sub_info->last_time_received = ros::Time(0);
            sub_info->rate = ros::Duration(0);
            sub_info->advertised = false;
            ROS_INFO_STREAM("subscribe " << sub_info->topic_name);
            sub_info->sub = new ros::Subscriber(n.subscribe<ShapeShifter>(sub_info->topic_name, 10, boost::bind(in_cb, _1, sub_info)));

            // waiting for all topics are publisherd
            while (sub_info->sub->getNumPublishers() == 0 ) {
                ros::Duration(1.0).sleep();
                ROS_WARN_STREAM("wait for " << sub_info->topic_name << " ... " << i << "/" << topics.size());
            }

            g_subs.push_back(sub_info);
        }
    ROS_INFO_STREAM("setup done, subscribed " << topics.size() << " topics");

    // New service
    ros::ServiceServer ss_list = nh.advertiseService(string("list"), list_topic_cb);

    ros::ServiceServer ss_update = nh.advertiseService(string("update"), update_topic_cb);
    ros::Subscriber ss_update_sub = nh.subscribe("update", 1, update_topic_cb);
    ros::spin();

    return 0;
}
