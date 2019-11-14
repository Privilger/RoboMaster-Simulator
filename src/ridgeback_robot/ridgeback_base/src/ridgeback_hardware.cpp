/**
 *
 *  \file
 *  \brief      Class representing Ridgeback hardware
 *  \author     Mike Purvis <mpurvis@clearpathrobotics.com>
 *  \author     Tony Baltovski <tbaltovski@clearpathrobotics.com>
 *  \copyright  Copyright (c) 2013, Clearpath Robotics, Inc.
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

#include <vector>
#include "boost/assign.hpp"
#include "boost/shared_ptr.hpp"
#include "ridgeback_base/ridgeback_hardware.h"
#include "puma_motor_driver/driver.h"
#include "puma_motor_driver/multi_driver_node.h"

namespace ridgeback_base
{

RidgebackHardware::RidgebackHardware(ros::NodeHandle& nh, ros::NodeHandle& pnh,
                                     puma_motor_driver::Gateway& gateway):
  nh_(nh),
  pnh_(pnh),
  gateway_(gateway),
  active_(false)
{
  pnh_.param<double>("gear_ratio", gear_ratio_, 34.97);
  pnh_.param<int>("encoder_cpr", encoder_cpr_, 1024);

  ros::V_string joint_names = boost::assign::list_of("front_left_wheel")
      ("front_right_wheel")("rear_left_wheel")("rear_right_wheel");
  std::vector<uint8_t> joint_canids = boost::assign::list_of(5)(4)(2)(3);
  std::vector<float> joint_directions = boost::assign::list_of(-1)(1)(-1)(1);

  for (uint8_t i = 0; i < joint_names.size(); i++)
  {
    hardware_interface::JointStateHandle joint_state_handle(joint_names[i],
        &joints_[i].position, &joints_[i].velocity, &joints_[i].effort);
    joint_state_interface_.registerHandle(joint_state_handle);

    hardware_interface::JointHandle joint_handle(
        joint_state_handle, &joints_[i].velocity_command);
    velocity_joint_interface_.registerHandle(joint_handle);

    puma_motor_driver::Driver driver(gateway_, joint_canids[i], joint_names[i]);
    driver.clearStatusCache();
    driver.setEncoderCPR(encoder_cpr_);
    driver.setGearRatio(gear_ratio_ * joint_directions[i]);
    driver.setMode(puma_motor_msgs::Status::MODE_SPEED, 0.1, 0.01, 0.0);
    drivers_.push_back(driver);
  }

  registerInterface(&joint_state_interface_);
  registerInterface(&velocity_joint_interface_);

  multi_driver_node_.reset(new puma_motor_driver::MultiDriverNode(nh_, drivers_));
}

/**
 * Populates the internal joint state struct from the most recent CAN data
 * received from the motor controller.
 *
 * Called from the controller thread.
 */
void RidgebackHardware::updateJointsFromHardware()
{
  uint8_t index = 0;
  BOOST_FOREACH(puma_motor_driver::Driver& driver, drivers_)
  {
    Joint* f = &joints_[index];
    f->effort = driver.lastCurrent();
    f->position = driver.lastPosition();
    f->velocity = driver.lastSpeed();
    index++;
  }
}

bool RidgebackHardware::isActive()
{
  if (active_ == false
     && drivers_[0].isConfigured() == true
     && drivers_[1].isConfigured() == true
     && drivers_[2].isConfigured() == true
     && drivers_[3].isConfigured() == true)
  {
    active_ = true;
    multi_driver_node_->activePublishers(active_);
    ROS_INFO("Ridgeback Hardware Active.");
  }
  else if ((drivers_[0].isConfigured() == false
    || drivers_[1].isConfigured() == false
    || drivers_[2].isConfigured() == false
    || drivers_[3].isConfigured() == false)
    && active_ == true)
  {
    active_ = false;
    ROS_WARN("Ridgeback Hardware Inactive.");
  }

  return active_;
}

void RidgebackHardware::requestData()
{
  BOOST_FOREACH(puma_motor_driver::Driver& driver, drivers_)
  {
    driver.requestFeedbackPowerState();
  }
}
void RidgebackHardware::powerHasNotReset()
{
  // Checks to see if power flag has been reset for each driver
  BOOST_FOREACH(puma_motor_driver::Driver& driver, drivers_)
  {
    if (driver.lastPower() != 0)
    {
      active_ = false;
      ROS_WARN("There was a power rest on Dev: %d, will reconfigure all drivers.", driver.deviceNumber());
      multi_driver_node_->activePublishers(active_);
      BOOST_FOREACH(puma_motor_driver::Driver& driver, drivers_)
      {
        driver.resetConfiguration();
      }
    }
  }
}

void RidgebackHardware::configure()
{
  BOOST_FOREACH(puma_motor_driver::Driver& driver, drivers_)
  {
    driver.configureParams();
  }
}
void RidgebackHardware::verify()
{
  BOOST_FOREACH(puma_motor_driver::Driver& driver, drivers_)
  {
    driver.verifyParams();
  }
}

bool RidgebackHardware::inReset()
{
  return !active_;
}

void RidgebackHardware::init()
{
  while (!connectIfNotConnected())
  {
    ros::Duration(1.0).sleep();
  }
}

void RidgebackHardware::canRead()
{
  // Process all received messages through the connected driver instances.
  puma_motor_driver::Message recv_msg;
  while (gateway_.recv(&recv_msg))
  {
    BOOST_FOREACH(puma_motor_driver::Driver& driver, drivers_)
    {
      driver.processMessage(recv_msg);
    }
  }
}

void RidgebackHardware::canSend()
{
  gateway_.sendAllQueued();
}

bool RidgebackHardware::connectIfNotConnected()
{
  if (!gateway_.isConnected())
  {
    if (!gateway_.connect())
    {
      ROS_ERROR("Error connecting to motor driver gateway. Retrying in 1 second.");
      return false;
    }
    else
    {
      ROS_INFO("Connection to motor driver gateway successful.");
    }
  }
  return true;
}

/**
 * Populates and publishes Drive message based on the controller outputs.
 *
 * Called from the controller thread.
 */
void RidgebackHardware::command()
{
  uint8_t i = 0;
  BOOST_FOREACH(puma_motor_driver::Driver& driver, drivers_)
  {
    driver.commandSpeed(joints_[i].velocity_command);
    i++;
  }
}

std::vector<puma_motor_driver::Driver>& RidgebackHardware::getDrivers()
{
    return drivers_;
}

}  // namespace ridgeback_base
