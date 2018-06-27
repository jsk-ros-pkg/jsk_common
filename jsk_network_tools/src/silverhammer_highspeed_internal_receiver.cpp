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

#include <iostream>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <jsk_network_tools/SilverhammerInternalBuffer.h>
#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <algorithm>

namespace jsk_network_tools
{
  typedef unsigned char Base;
  //typedef char Base;
  class Packet
  {
  public:
    typedef boost::shared_ptr<Packet> Ptr;
    size_t seq_id;
    size_t packet_id;
    size_t packet_num;
    std::vector<Base> data;
    Packet(size_t aseq_id, size_t apacket_id, size_t apacket_num):
      seq_id(aseq_id), packet_id(apacket_id), packet_num(apacket_num)
    {
    }

    void fillData(size_t size, Base* buf)
    {
      data.resize(size);
      memcpy(&data[0], &buf[12], size);
    }

    size_t fill(std::vector<Base>& target, size_t start, size_t end)
    {
      memcpy(&target[start], &data[0], std::min(end - start, data.size()));
      return std::min(end - start, data.size());
    }

    bool operator<(const Packet& other) const
    {
      return packet_id < other.packet_id;
    }
  };

  class SilverhammerHighspeedInternalReceiver
  {
  public:
    typedef std::vector<Packet> PacketArray;
    typedef std::map<size_t, PacketArray> PacketTable;
    SilverhammerHighspeedInternalReceiver(): initialized_(false)
    {
      ros::NodeHandle nh, pnh("~");
      nh.param("receive_port", receive_port_, 16484);
      nh.param("pesimistic", pesimistic_, true);
      nh.param("fragment_packets_tolerance", fragment_packets_tolerance_, 0);
      nh.param("expected_rate", expected_rate_, 2.0);
      pub_ = nh.advertise<SilverhammerInternalBuffer>("input", 1);
      // nh.param("packet_size", packet_size_, 1400);
      boost::asio::io_service io_service;
      socket_.reset(new boost::asio::ip::udp::socket(io_service, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), receive_port_)));
      thread_ = boost::thread(boost::bind(&SilverhammerHighspeedInternalReceiver::threadFunc, this));
      pub_thread_ = boost::thread(boost::bind(&SilverhammerHighspeedInternalReceiver::publishPacketsFunc, this));
      
    }

  protected:

    void threadFunc()
    {
      PacketArray packet_array;
      // Is it fast enough?
      int last_received_seq_id = -1;
      while (ros::ok()) {
        boost::array<Base, 1500> recv_buf; // only support 1500 MTU (without header)
        boost::asio::ip::udp::endpoint remote_endpoint;
        boost::system::error_code error;
        size_t len = socket_->receive_from(boost::asio::buffer(recv_buf), remote_endpoint, 0, error);
        //ROS_INFO("received packet (%lu bytes = %lu bits)", len, len * 8);
        if (len < 4 + 4 + 4) {
          ROS_WARN("too short packet");
          continue;
        }
        size_t seq_id = recv_buf[0] * (1 << 24) + recv_buf[1] * (1 << 16) + recv_buf[2] * (1 << 8) + recv_buf[3] * (1 << 0);
        size_t packet_id = recv_buf[0 + 4] * (1 << 24) + recv_buf[1 + 4] * (1 << 16) + recv_buf[2 + 4] * (1 << 8) + recv_buf[3 + 4] * (1 << 0);
        size_t packet_num = recv_buf[0 + 8] * (1 << 24) + recv_buf[1 + 8] * (1 << 16) + recv_buf[2 + 8] * (1 << 8) + recv_buf[3 + 8] * (1 << 0);
        
        Packet packet(seq_id, packet_id, packet_num);
        packet.fillData(len - 4 * 3, recv_buf.data());
        // ROS_INFO("seq_id := %lu", seq_id);
        //ROS_INFO("packet_id := %lu", packet_id);
        // ROS_INFO("packet_num := %lu", packet_num);

        if (last_received_seq_id == -1) {
          packet_array.push_back(packet);
          last_received_seq_id = seq_id;
        }
        else if (last_received_seq_id != seq_id) {
          ROS_INFO("seq_id := %d, packet_array.size() := %lu, packet_num := %lu",
                   last_received_seq_id, packet_array.size(), packet_array[0].packet_num);
          size_t before_size = packet_array.size();
          if (packet_array.size() == packet_array[0].packet_num || !pesimistic_) {
            boost::mutex::scoped_lock lock(shared_packet_mutex_);
            shared_packet_array_ = packet_array;
            // Exit from special section as soon as possible
          }
          packet_array = PacketArray();
          packet_array.reserve(before_size * 2);
          packet_array.push_back(packet);
        }
        else {
          packet_array.push_back(packet);
        }
        last_received_seq_id = seq_id;
      }
    }

    void publishPacketsFunc()
    {
      ros::Rate r(expected_rate_);
      int published_seq_id = -1;
      while (ros::ok()) {
        {
          PacketArray publish_thread_packet_array;
          {
            boost::mutex::scoped_lock lock(shared_packet_mutex_);
            if (shared_packet_array_.size() != 0) {
              publish_thread_packet_array = shared_packet_array_;
            }
            // Exit from special section as soon as possible
          }
          if (publish_thread_packet_array.size() != 0) {
            if (publish_thread_packet_array[0].seq_id != published_seq_id) {
              publishPackets(publish_thread_packet_array);
              published_seq_id = publish_thread_packet_array[0].seq_id;
            }
          }
          r.sleep();
        }
      }
    }

    void publishPackets(PacketArray& packet_array)
    {
      msg_.header.stamp = ros::Time::now();
      msg_.seq_id = packet_array[0].seq_id;
      // Sort packet_array according to packet_id
      std::sort(packet_array.begin(), packet_array.end());
      // We expect all the packet size are same
      size_t packet_size = packet_array[0].data.size();
      size_t num = packet_array[0].packet_num;
      if (msg_.data.size() != num * packet_size) {
        msg_.data.resize(num * packet_size);
      }
      std::fill(msg_.data.begin(), msg_.data.end(), 0);
      PacketArray::iterator it = packet_array.begin();
      size_t target_packet_id = 0;
      
      ROS_INFO("expected data size: %lu", num * packet_size);
      size_t len = 0;
      while (target_packet_id < num && it != packet_array.end()) {
        if (it->packet_id == target_packet_id) {
          len += it->fill(msg_.data, target_packet_id * packet_size, (target_packet_id + 1) * packet_size);
          ++it;
        }
        else {                  // fill by dummy 0 data == just skip
          // pass
          len += packet_size;
        }
        ++target_packet_id;
      }
      ROS_INFO("len: %lu", len);
      msg_.data.resize(len);
      pub_.publish(msg_);
    }

    PacketArray shared_packet_array_;
    SilverhammerInternalBuffer msg_;
    bool initialized_;
    boost::mutex mutex_;
    boost::mutex shared_packet_mutex_;
    boost::condition_variable thread_state_;
    boost::thread thread_;
    boost::thread pub_thread_;
    boost::shared_ptr<boost::asio::ip::udp::socket> socket_;
    int receive_port_;
    int fragment_packets_tolerance_;
    std::string receive_ip_;
    double expected_rate_;
    //int packet_size_;
    ros::Publisher pub_;
    bool pesimistic_;
  private:
  };
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "silverhammer_highspeed_internal_receiver");
  jsk_network_tools::SilverhammerHighspeedInternalReceiver receiver;
  ros::spin();
  return 0;
}
