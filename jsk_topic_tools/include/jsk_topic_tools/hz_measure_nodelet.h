// -*- mode: c++ -*->
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Ryohei Ueda and JSK Lab
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

#ifndef JSK_TOPIC_TOOLS_HZ_MEASURE_NODELET_H_
#define JSK_TOPIC_TOOLS_HZ_MEASURE_NODELET_H_

#include <nodelet/nodelet.h>
#include <topic_tools/shape_shifter.h>

#include "jsk_topic_tools/timered_diagnostic_updater.h"

#include <queue>

namespace jsk_topic_tools
{
  class HzMeasure: public nodelet::Nodelet
  {
  public:
    typedef ros::MessageEvent<topic_tools::ShapeShifter> ShapeShifterEvent;
    virtual void onInit();
  protected:
    int average_message_num_;
    double measure_time_;
    double warning_hz_;
    std::queue<ros::Time> buffer_;
    ros::Publisher hz_pub_;
    ros::Subscriber sub_;
    ros::NodeHandle pnh_;
    virtual void popBufferQueue();
    virtual double calculateHz();
    virtual void inputCallback(const boost::shared_ptr<topic_tools::ShapeShifter const>& msg);

    /** @brief
     * Method which is called periodically.
     *
     * In default, it check vitality of vital_checker_ and if vital_checker_
     * is not poked for seconds, diagnostic status will be ERROR.
     * @param stat Modofy stat to change status of diagnostic information.
     */
    virtual void updateDiagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat);

    /** @brief
     * True if summary is displayed as 'Warnings', otherwise it is displayed as 'Errors'
     */
    uint8_t diagnostic_error_level_;

    /** @brief
     * Pointer to TimeredDiagnosticUpdater to call
     * updateDiagnostic(diagnostic_updater::DiagnosticStatusWrapper&)
     * periodically.
     */
    TimeredDiagnosticUpdater::Ptr diagnostic_updater_;
  private:
  };
}

#endif
