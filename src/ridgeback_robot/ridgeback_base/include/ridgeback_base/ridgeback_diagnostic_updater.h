/**
 *
 *  \file
 *  \brief      Diagnostic updating class for Ridgeback
 *  \author     Mike Purvis <mpurvis@clearpathrobotics.com>
 *  \copyright  Copyright (c) 2014, Clearpath Robotics, Inc.
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

#ifndef RIDGEBACK_BASE_RIDGEBACK_DIAGNOSTIC_UPDATER_H
#define RIDGEBACK_BASE_RIDGEBACK_DIAGNOSTIC_UPDATER_H

#include <string>

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/Imu.h"
#include "diagnostic_updater/diagnostic_updater.h"
#include "diagnostic_updater/publisher.h"
#include "ridgeback_msgs/Status.h"

namespace ridgeback_base
{

class RidgebackDiagnosticUpdater : private diagnostic_updater::Updater
{
public:
  RidgebackDiagnosticUpdater();

  void generalDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);
  void batteryDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);
  void voltageDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);
  void currentDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);
  void powerDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);
  void temperatureDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);

  void statusCallback(const ridgeback_msgs::Status::ConstPtr& status);
  void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
  void wirelessMonitorCallback(const ros::TimerEvent& te);

private:
  ros::NodeHandle nh_;
  ros::Timer timer_;
  ros::Subscriber status_sub_;
  ridgeback_msgs::Status::ConstPtr last_status_;

  double expected_imu_frequency_;
  diagnostic_updater::TopicDiagnostic* imu_diagnostic_;
  ros::Subscriber imu_sub_;

  char hostname_[1024];

  std::string wireless_interface_;
  ros::Timer wireless_monitor_timer_;
  ros::Publisher wifi_connected_pub_;
};

}  // namespace ridgeback_base

#endif  // RIDGEBACK_BASE_RIDGEBACK_DIAGNOSTIC_UPDATER_H
