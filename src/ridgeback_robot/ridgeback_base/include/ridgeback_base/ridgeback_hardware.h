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

#ifndef RIDGEBACK_BASE_RIDGEBACK_HARDWARE_H
#define RIDGEBACK_BASE_RIDGEBACK_HARDWARE_H

#include <vector>

#include "boost/thread.hpp"
#include "boost/foreach.hpp"
#include "boost/shared_ptr.hpp"
#include "hardware_interface/joint_state_interface.h"
#include "hardware_interface/joint_command_interface.h"
#include "hardware_interface/robot_hw.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "puma_motor_driver/socketcan_gateway.h"
#include "puma_motor_driver/driver.h"
#include "puma_motor_driver/multi_driver_node.h"
#include "puma_motor_msgs/MultiFeedback.h"

namespace ridgeback_base
{

class RidgebackHardware : public hardware_interface::RobotHW
{
public:
  RidgebackHardware(ros::NodeHandle& nh, ros::NodeHandle& pnh,
                    puma_motor_driver::Gateway& gateway);
  void init();  // Connect to CAN
  bool connectIfNotConnected();  // Keep trying till it connects
  std::vector<puma_motor_driver::Driver>& getDrivers();
  void configure();  // Configures the motor drivers
  void verify();
  bool isActive();

  void powerHasNotReset();  // Checks if power has been reset
  bool inReset();  // Returns if the cm should be reset based on the state of the motors drivers.
                   // If they have been configured.
  void requestData();
  void updateJointsFromHardware();
  void command();

  void canSend();
  void canRead();

private:
  ros::NodeHandle nh_, pnh_;

  puma_motor_driver::Gateway& gateway_;
  std::vector<puma_motor_driver::Driver> drivers_;
  boost::shared_ptr<puma_motor_driver::MultiDriverNode> multi_driver_node_;

  bool active_;
  double gear_ratio_;
  int encoder_cpr_;

  // ROS Control interfaces
  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::VelocityJointInterface velocity_joint_interface_;

  // These are mutated on the controls thread only.
  struct Joint
  {
    double position;
    double velocity;
    double effort;
    double velocity_command;

    Joint() : position(0), velocity(0), effort(0), velocity_command(0)
    {
    }
  }
  joints_[4];
};

}  // namespace ridgeback_base

#endif  // RIDGEBACK_BASE_RIDGEBACK_HARDWARE_H
