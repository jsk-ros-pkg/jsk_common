// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, JSK Lab
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


#include <string>
#include <sstream>
#include <multisense_lib/MultiSenseChannel.hh>
#include <iostream>
#include <stdlib.h>

int main (int argc, char **argv)
{
  if (argc < 3) {
    std::cerr << "Usage: " << argv[0] << " multisense_ip pc_ip" << std::endl;
  }
  else {
    std::string multisense_ip = argv[1];
    std::string pc_ip = argv[2];
    // Pinging to pc_ip
    std::stringstream command;
    command << "ping -c 1 " << pc_ip << " > /dev/null";
    int missing_counter = 0;
    while (true) {
      if (system(command.str().c_str()) != 0) {
        ++missing_counter;
        std::cout << "increasing missing counter: " << missing_counter << std::endl;
      }
      else {
        missing_counter = 0;
      }
      if (missing_counter > 10) {
        crl::multisense::Channel *channelP
          = crl::multisense::Channel::Create(multisense_ip); 
        if (channelP) {
          std::cerr << "killing multisense[" << multisense_ip << "]" << std::endl;
          channelP->stopStreams(crl::multisense::Source_All); 
          crl::multisense::Channel::Destroy(channelP);
          missing_counter = 0;
        }
      }
    }
  }
  return 1;
}
