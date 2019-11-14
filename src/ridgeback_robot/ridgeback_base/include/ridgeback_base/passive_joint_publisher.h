/**
 *
 *  \file
 *  \brief      Class publishing the joint state for passive front axle.
 *  \author     Mike Purvis <mpurvis@clearpathrobotics.com>
 *  \copyright  Copyright (c) 2015, Clearpath Robotics, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Clearpath Robotics, Inc. nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Please send comments, questions, or patches to code@clearpathrobotics.com
 *
 */

#ifndef RIDGEBACK_BASE_PASSIVE_JOINT_PUBLISHER_H
#define RIDGEBACK_BASE_PASSIVE_JOINT_PUBLISHER_H

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

namespace ridgeback_base
{

class PassiveJointPublisher
{
public:
  PassiveJointPublisher(ros::NodeHandle& nh, ros::V_string& joints, int frequency)
  {
    for (int i = 0; i < joints.size(); i++)
    {
      msg_.name.push_back(joints[i]);
      msg_.position.push_back(0);
      msg_.velocity.push_back(0);
      msg_.effort.push_back(0);
    }
    pub_ = nh.advertise<sensor_msgs::JointState>("/joint_states", 1);
    timer_ = nh.createTimer(ros::Duration(1.0/frequency), &PassiveJointPublisher::timerCb, this);
  }

  void timerCb(const ros::TimerEvent&)
  {
    msg_.header.stamp = ros::Time::now();
    pub_.publish(msg_);
  }

private:
  sensor_msgs::JointState msg_;
  ros::Publisher pub_;
  ros::Timer timer_;
};

}  // namespace ridgeback_base

#endif  // RIDGEBACK_BASE_PASSIVE_JOINT_PUBLISHER_H
