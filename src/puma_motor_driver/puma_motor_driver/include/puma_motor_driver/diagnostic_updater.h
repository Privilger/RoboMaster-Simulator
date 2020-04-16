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
#ifndef PUMA_MOTOR_DRIVER_DIAGNOSTIC_UPDATER_H
#define PUMA_MOTOR_DRIVER_DIAGNOSTIC_UPDATER_H

#include <string>

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "diagnostic_updater/diagnostic_updater.h"
#include "diagnostic_updater/publisher.h"
#include "puma_motor_msgs/MultiStatus.h"
#include "puma_motor_msgs/Status.h"

namespace puma_motor_driver
{

class PumaMotorDriverDiagnosticUpdater : private diagnostic_updater::Updater
{
public:
  PumaMotorDriverDiagnosticUpdater();

  void driverDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat, int driver);

  void statusCallback(const puma_motor_msgs::MultiStatus::ConstPtr& status_msg);

private:
  ros::NodeHandle nh_;
  ros::Subscriber status_sub_;
  puma_motor_msgs::MultiStatus::ConstPtr last_status_;
  bool initialized_;

  static const char* getFaultString(uint8_t fault);
  static const char* getModeString(uint8_t mode);
};

}  // namespace puma_motor_driver

#endif  // PUMA_MOTOR_DRIVER_DIAGNOSTIC_UPDATER_H
