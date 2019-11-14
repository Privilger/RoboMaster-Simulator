/**
Software License Agreement (BSD)

\file      ridgeback_lighting.cpp
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

#include "ridgeback_base/ridgeback_lighting.h"
#include <boost/assign/list_of.hpp>

namespace ridgeback_base
{

namespace States
{
  enum State
  {
    Idle = 0,
    Driving,
    Charged,
    Charging,
    LowBattery,
    NeedsReset,
    Fault,
    Stopped,
    NumberOfStates
  };
}  // namespace States
typedef States::State State;

namespace Colours
{
  enum Colour
  {
    Off = 0x000000,
    Red_H = 0xFF0000,
    Red_M = 0xAA00000,
    Red_L = 0x550000,
    Green_H = 0x00FF00,
    Green_M = 0x00AA00,
    Green_L = 0x005500,
    Blue_H = 0x0000FF,
    Blue_M = 0x0000AA,
    Blue_L = 0x000055,
    Yellow_H = 0xFFFF00,
    Yellow_M = 0xAAAA00,
    Yellow_L = 0x555500,
    Orange_H = 0xFFAE00,
    Orange_M = 0xF0A80E,
    Orange_L = 0xD69F27,
    White_H = 0xFFFFFF,
    White_M = 0xAAAAAA,
    White_L = 0x555555
  };
}  // namespace Colours
typedef Colours::Colour Colour;


RidgebackLighting::RidgebackLighting(ros::NodeHandle* nh) :
  nh_(nh),
  allow_user_(false),
  user_publishing_(false),
  state_(States::Idle),
  current_pattern_count_(0)
{
  lights_pub_ = nh_->advertise<ridgeback_msgs::Lights>("mcu/cmd_lights", 1);

  user_cmds_sub_ = nh_->subscribe("cmd_lights", 1, &RidgebackLighting::userCmdCallback, this);
  mcu_status_sub_ = nh_->subscribe("mcu/status", 1, &RidgebackLighting::mcuStatusCallback, this);
  puma_status_sub_ = nh_->subscribe("status", 1, &RidgebackLighting::pumaStatusCallback, this);
  cmd_vel_sub_ = nh_->subscribe("cmd_vel", 1, &RidgebackLighting::cmdVelCallback, this);

  pub_timer_ = nh_->createTimer(ros::Duration(1.0/5), &RidgebackLighting::timerCb, this);
  user_timeout_ = nh_->createTimer(ros::Duration(1.0/1), &RidgebackLighting::userTimeoutCb, this);

  using namespace Colours;  // NOLINT(build/namespaces)
  patterns_.stopped.push_back(boost::assign::list_of(Red_H)(Red_H)(Red_H)(Red_H)(Red_H)(Red_H)(Red_H)(Red_H));
  patterns_.stopped.push_back(boost::assign::list_of(Off)(Off)(Off)(Off)(Off)(Off)(Off)(Off));

  patterns_.fault.push_back(
    boost::assign::list_of(Orange_H)(Orange_H)(Orange_H)(Orange_H)(Orange_H)(Orange_H)(Orange_H)(Orange_H));
  patterns_.fault.push_back(boost::assign::list_of(Off)(Off)(Off)(Off)(Off)(Off)(Off)(Off));

  patterns_.reset.push_back(boost::assign::list_of(Off)(Red_H)(Off)(Red_H)(Yellow_H)(Yellow_H)(Red_H)(Off));
  patterns_.reset.push_back(boost::assign::list_of(Red_H)(Off)(Red_H)(Off)(Red_H)(Red_H)(Off)(Red_H));

  patterns_.low_battery.push_back(
    boost::assign::list_of(Orange_L)(Orange_L)(Orange_L)(Orange_L)(Orange_L)(Orange_L)(Orange_L)(Orange_L));
  patterns_.low_battery.push_back(
    boost::assign::list_of(Orange_M)(Orange_M)(Orange_M)(Orange_M)(Orange_M)(Orange_M)(Orange_M)(Orange_M));
  patterns_.low_battery.push_back(
    boost::assign::list_of(Orange_H)(Orange_H)(Orange_H)(Orange_H)(Orange_H)(Orange_H)(Orange_H)(Orange_H));
  patterns_.low_battery.push_back(
    boost::assign::list_of(Orange_M)(Orange_M)(Orange_M)(Orange_M)(Orange_M)(Orange_M)(Orange_M)(Orange_M));
  patterns_.low_battery.push_back(
    boost::assign::list_of(Orange_L)(Orange_L)(Orange_L)(Orange_L)(Orange_L)(Orange_L)(Orange_L)(Orange_L));

  patterns_.charged.push_back(
    boost::assign::list_of(Green_H)(Green_H)(Green_H)(Green_H)(Green_H)(Green_H)(Green_H)(Green_H));

  patterns_.charging.push_back(
    boost::assign::list_of(Green_L)(Green_L)(Green_L)(Green_L)(Green_L)(Green_L)(Green_L)(Green_L));
  patterns_.charging.push_back(
    boost::assign::list_of(Green_M)(Green_M)(Green_M)(Green_M)(Green_M)(Green_M)(Green_M)(Green_M));
  patterns_.charging.push_back(
    boost::assign::list_of(Green_H)(Green_H)(Green_H)(Green_H)(Green_H)(Green_H)(Green_H)(Green_H));
  patterns_.charging.push_back(
    boost::assign::list_of(Green_M)(Green_M)(Green_M)(Green_M)(Green_M)(Green_M)(Green_M)(Green_M));
  patterns_.charging.push_back(
    boost::assign::list_of(Green_L)(Green_L)(Green_L)(Green_L)(Green_L)(Green_L)(Green_L)(Green_L));

  patterns_.driving.push_back(
    boost::assign::list_of(White_M)(White_M)(White_M)(White_M)(Red_M)(Red_M)(Red_M)(Red_M));

  patterns_.idle.push_back(
    boost::assign::list_of(White_L)(White_L)(White_L)(White_L)(Red_L)(Red_L)(Red_L)(Red_L));
}


void RidgebackLighting::setRGB(ridgeback_msgs::RGB* rgb, uint32_t colour)
{
  rgb->red = ((colour & 0xFF0000) >> 16) / 255.0;
  rgb->green = ((colour & 0x00FF00) >> 8) / 255.0;
  rgb->blue = ((colour & 0x0000FF)) / 255.0;
}

void RidgebackLighting::setLights(ridgeback_msgs::Lights* lights, uint32_t pattern[8])
{
  for (int i = 0; i < 8; i++)
  {
    setRGB(&lights->lights[i], pattern[i]);
  }
}

void RidgebackLighting::userCmdCallback(const ridgeback_msgs::Lights::ConstPtr& lights_msg)
{
  if (allow_user_)
  {
    lights_pub_.publish(lights_msg);
  }
  user_publishing_ = true;
}

void RidgebackLighting::mcuStatusCallback(const ridgeback_msgs::Status::ConstPtr& status_msg)
{
  mcu_status_msg_ = *status_msg;
}

void RidgebackLighting::pumaStatusCallback(const puma_motor_msgs::MultiStatus::ConstPtr& status_msg)
{
  pumas_status_msg_ = *status_msg;
}

void RidgebackLighting::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  cmd_vel_msg_ = *msg;
}

void RidgebackLighting::timerCb(const ros::TimerEvent&)
{
  updateState();

  if (state_ >= States::Charged)
  {
    allow_user_ = false;
  }
  else
  {
    allow_user_ = true;
  }

  if (!user_publishing_ || !allow_user_)
  {
    ridgeback_msgs::Lights lights_msg;
    updatePattern();
    setLights(&lights_msg, &current_pattern_[0]);
    lights_pub_.publish(lights_msg);
  }
}

void RidgebackLighting::userTimeoutCb(const ros::TimerEvent&)
{
  user_publishing_ = false;
}

void RidgebackLighting::updateState()
{
  if (mcu_status_msg_.stop_engaged == true)
  {
    state_ = States::Stopped;
  }
  else if (mcu_status_msg_.drivers_active == false)
  {
    state_ = States::NeedsReset;
  }
  else if (pumas_status_msg_.drivers.size() == 4 &&
          (pumas_status_msg_.drivers[0].fault != 0 ||
           pumas_status_msg_.drivers[1].fault != 0 ||
           pumas_status_msg_.drivers[2].fault != 0 ||
           pumas_status_msg_.drivers[3].fault != 0))
  {
    state_ = States::Fault;
  }
  else if (mcu_status_msg_.measured_battery <= 24.0)
  {
    state_ = States::LowBattery;
  }
  else if (mcu_status_msg_.charging_complete == true)
  {
    state_ = States::Charged;
  }
  else if (mcu_status_msg_.charger_connected == true)
  {
    state_ = States::Charging;
  }
  else if (cmd_vel_msg_.linear.x != 0.0 ||
           cmd_vel_msg_.linear.y != 0.0 ||
           cmd_vel_msg_.angular.z != 0.0)
  {
    state_ = States::Driving;
  }
  else
  {
    state_ = States::Idle;
  }
}

void RidgebackLighting::updatePattern()
{
  if (old_state_ != state_)
  {
    current_pattern_count_ = 0;
  }

  switch (state_)
  {
    case States::Stopped:
      if (current_pattern_count_ >= patterns_.stopped.size())
      {
        current_pattern_count_ = 0;
      }
      memcpy(&current_pattern_, &patterns_.stopped[current_pattern_count_], sizeof(current_pattern_));
      break;
    case States::Fault:
      if (current_pattern_count_ >= patterns_.fault.size())
      {
        current_pattern_count_ = 0;
      }
      memcpy(&current_pattern_, &patterns_.fault[current_pattern_count_], sizeof(current_pattern_));
      break;
    case States::NeedsReset:
      if (current_pattern_count_ >= patterns_.reset.size())
      {
        current_pattern_count_ = 0;
      }
      memcpy(&current_pattern_, &patterns_.reset[current_pattern_count_], sizeof(current_pattern_));
      break;
    case States::LowBattery:
      if (current_pattern_count_ >= patterns_.low_battery.size())
      {
        current_pattern_count_ = 0;
      }
      memcpy(&current_pattern_, &patterns_.low_battery[current_pattern_count_], sizeof(current_pattern_));
      break;
    case States::Charged:
      if (current_pattern_count_ >= patterns_.charged.size())
      {
        current_pattern_count_ = 0;
      }
      memcpy(&current_pattern_, &patterns_.charged[current_pattern_count_], sizeof(current_pattern_));
      break;
    case States::Charging:
      if (current_pattern_count_ >= patterns_.charging.size())
      {
        current_pattern_count_ = 0;
      }
      memcpy(&current_pattern_, &patterns_.charging[current_pattern_count_], sizeof(current_pattern_));
      break;
    case States::Driving:
      if (current_pattern_count_ >= patterns_.driving.size())
      {
        current_pattern_count_ = 0;
      }
      memcpy(&current_pattern_, &patterns_.driving[current_pattern_count_], sizeof(current_pattern_));
      break;
    case States::Idle:
      if (current_pattern_count_ >= patterns_.idle.size())
      {
        current_pattern_count_ = 0;
      }
      memcpy(&current_pattern_, &patterns_.idle[current_pattern_count_], sizeof(current_pattern_));
      break;
  }
  old_state_ = state_;
  current_pattern_count_++;
}

}  // namespace ridgeback_base
