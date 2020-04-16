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
#include <string>

#include "boost/foreach.hpp"
#include "diagnostic_updater/update_functions.h"
#include "puma_motor_driver/diagnostic_updater.h"
#include "puma_motor_msgs/Status.h"

namespace puma_motor_driver
{

  typedef diagnostic_msgs::DiagnosticStatus Status;

PumaMotorDriverDiagnosticUpdater::PumaMotorDriverDiagnosticUpdater()
{
  initialized_ = false;
  setHardwareID("none");
  status_sub_ = nh_.subscribe("status", 5, &PumaMotorDriverDiagnosticUpdater::statusCallback, this);
}

const char* PumaMotorDriverDiagnosticUpdater::getModeString(uint8_t mode)
{
  switch (mode)
  {
    case puma_motor_msgs::Status::MODE_VOLTAGE:
      return "Voltage Control";
    case puma_motor_msgs::Status::MODE_CURRENT:
      return "Current Control";
    case puma_motor_msgs::Status::MODE_SPEED:
      return "Speed control";
    case puma_motor_msgs::Status::MODE_POSITION:
      return "Position control";
    case puma_motor_msgs::Status::MODE_VCOMP:
      return "Vcomp control";
    default:
      return "Unknown control";
  }
}

const char* PumaMotorDriverDiagnosticUpdater::getFaultString(uint8_t fault)
{
  switch (fault)
  {
    case puma_motor_msgs::Status::FAULT_CURRENT:
      return "current fault";
    case puma_motor_msgs::Status::FAULT_TEMPERATURE:
      return "temperature fault";
    case puma_motor_msgs::Status::FAULT_BUS_VOLTAGE:
      return "bus voltage failt";
    case puma_motor_msgs::Status::FAULT_BRIDGE_DRIVER:
      return "bridge driver fault";
    default:
      return "unknown fault";
  }
}

void PumaMotorDriverDiagnosticUpdater::driverDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat, int driver)
{
  if (last_status_->drivers[driver].fault == 0)
  {
    stat.summary(Status::OK, "Motor driver is OK.");
  }
  else
  {
    stat.summaryf(Status::ERROR, "'%s' driver (%i) has a %s.",
      (last_status_->drivers[driver].device_name.c_str()),
       last_status_->drivers[driver].device_number,
       getFaultString(last_status_->drivers[driver].fault));
  }

  stat.add("Driver CAN ID", static_cast<int>(last_status_->drivers[driver].device_number));
  stat.add("Driver Role", last_status_->drivers[driver].device_name.c_str());
  stat.add("Driver Mode", getModeString(last_status_->drivers[driver].mode));

  stat.add("Input terminal voltage (V)", last_status_->drivers[driver].bus_voltage);
  stat.add("Internal driver temperature (degC)", last_status_->drivers[driver].temperature);
  stat.add("Voltage as output to the motor (V)", last_status_->drivers[driver].output_voltage);
  stat.add("Value of the auxiliary ADC (V)", last_status_->drivers[driver].analog_input);
}

void PumaMotorDriverDiagnosticUpdater::statusCallback(const puma_motor_msgs::MultiStatus::ConstPtr& status_msg)
{
  last_status_ = status_msg;
  if (!initialized_)
  {
    for (int i = 0; i < status_msg->drivers.size(); i++)
    {
      char name[100];
      snprintf(name, sizeof(name), "Puma motor driver on: %s with CAN ID (%d)",
        last_status_->drivers[i].device_name.c_str(), last_status_->drivers[i].device_number);
      add(name, boost::bind(&PumaMotorDriverDiagnosticUpdater::driverDiagnostics, this, _1, i));
    }
    initialized_ = true;
  }
  else
  {
    update();
  }
}

}  // namespace puma_motor_driver
