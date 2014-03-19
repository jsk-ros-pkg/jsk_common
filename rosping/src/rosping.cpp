// doc/html/boost_asio/example/icmp/

#include "ping.cpp"
#include <ros/ros.h>
#include <std_msgs/Float64.h>

int main(int argc, char* argv[])
{
  try
      {
          ros::init(argc, argv, "ping");
          if (argc != 2)
              {
                  std::cerr << "Usage: " << argv[0] << " <host>" << std::endl;
#if !defined(BOOST_WINDOWS)
                  std::cerr << "(You may need to set permission to run this program as root.)\n$ chown root.root rosping; ls -al rosping; chmod 4755 rosping\n" << std::endl;
#endif
                  return 1;
              }

          boost::asio::io_service io_service;
          pinger p(io_service, argv[1]);

          

          ros::NodeHandle n;
          ros::NodeHandle pnh("~");
          double rate;
          if (!pnh.getParam("rate", rate)) {
            rate = 10.0;         // 0.1Hz
          }
          ros::Publisher pub = n.advertise<std_msgs::Float64>("ping/delay", 10);

          std_msgs::Float64 msg;

          ros::Time last_time = ros::Time::now();

          while (ros::ok()) {
              ros::Time now = ros::Time::now();
              io_service.run_one();              
              if ( now - last_time > ros::Duration(rate) ) {

                  msg.data = p.delay;
                  pub.publish(msg);
                  ros::spinOnce();

                  last_time = now;
              }
          }

      }
  catch (std::exception& e)
      {
          std::cerr << "Exception: " << e.what() << std::endl;
      }
}
