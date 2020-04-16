/**
Software License Agreement (BSD)

\authors   Tony Baltovski <tbaltovski@clearpathrobotics.com>
\copyright Copyright (c) 2015, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that
the following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the
   following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
   following disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or promote
   products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WAR-
RANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, IN-
DIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include "puma_motor_driver/multi_driver_node.h"
#include "puma_motor_driver/driver.h"
#include "puma_motor_msgs/MultiStatus.h"
#include "puma_motor_msgs/Status.h"
#include "puma_motor_msgs/MultiFeedback.h"
#include "puma_motor_msgs/Feedback.h"
#include "boost/foreach.hpp"
#include <cstring>
#include <vector>
#include <ros/ros.h>

namespace puma_motor_driver
{

MultiDriverNode::MultiDriverNode(ros::NodeHandle& nh, std::vector<puma_motor_driver::Driver>& drivers)
  : nh_(nh), drivers_(drivers), active_(false)
  {
    feedback_pub_ = nh_.advertise<puma_motor_msgs::MultiFeedback>("feedback", 5);
    status_pub_ = nh_.advertise<puma_motor_msgs::MultiStatus>("status", 5);

    feedback_msg_.drivers_feedback.resize(drivers_.size());
    status_msg_.drivers.resize(drivers_.size());

    feedback_pub_timer_ = nh_.createTimer(ros::Duration(1.0/25), &MultiDriverNode::feedbackTimerCb, this);
    status_pub_timer_ = nh_.createTimer(ros::Duration(1.0/1), &MultiDriverNode::statusTimerCb, this);
  }

void MultiDriverNode::publishFeedback()
{
  // Prepare output feedback message to ROS.
  uint8_t feedback_index = 0;
  BOOST_FOREACH(puma_motor_driver::Driver& driver, drivers_)
  {
    puma_motor_msgs::Feedback* f = &feedback_msg_.drivers_feedback[feedback_index];
    f->device_number = driver.deviceNumber();
    f->device_name = driver.deviceName();
    f->duty_cycle = driver.lastDutyCycle();
    f->current = driver.lastCurrent();
    f->travel = driver.lastPosition();
    f->speed = driver.lastSpeed();
    f->setpoint = driver.lastSetpoint();

    feedback_index++;
  }
  feedback_msg_.header.stamp = ros::Time::now();
  feedback_pub_.publish(feedback_msg_);
}

void MultiDriverNode::publishStatus()
{
  // Prepare output status message to ROS.
  uint8_t status_index = 0;
  BOOST_FOREACH(puma_motor_driver::Driver& driver, drivers_)
  {
    puma_motor_msgs::Status* s = &status_msg_.drivers[status_index];
    s->device_number = driver.deviceNumber();
    s->device_name = driver.deviceName();
    s->bus_voltage = driver.lastBusVoltage();
    s->output_voltage = driver.lastOutVoltage();
    s->analog_input = driver.lastAnalogInput();
    s->temperature = driver.lastTemperature();
    s->mode = driver.lastMode();
    s->fault = driver.lastFault();

    status_index++;
  }
  status_msg_.header.stamp = ros::Time::now();
  status_pub_.publish(status_msg_);
}

void MultiDriverNode::statusTimerCb(const ros::TimerEvent&)
{
  if (active_)
  {
    publishStatus();
  }
}

void MultiDriverNode::feedbackTimerCb(const ros::TimerEvent&)
{
  if (active_)
  {
    publishFeedback();
  }
}

void MultiDriverNode::activePublishers(bool activate)
{
  active_ = activate;
}

}  // namespace puma_motor_driver
