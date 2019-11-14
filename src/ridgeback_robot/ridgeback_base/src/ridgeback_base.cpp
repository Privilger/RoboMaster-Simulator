/**
 *
 *  \file
 *  \brief      Main entry point for ridgeback base.
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


#include <string>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <boost/foreach.hpp>
#include <boost/assign/list_of.hpp>

#include "controller_manager/controller_manager.h"
#include "ridgeback_base/ridgeback_diagnostic_updater.h"
#include "ridgeback_base/ridgeback_hardware.h"
#include "ridgeback_base/ridgeback_cooling.h"
#include "ridgeback_base/ridgeback_lighting.h"
#include "ridgeback_base/passive_joint_publisher.h"
#include "puma_motor_driver/diagnostic_updater.h"
#include "ros/ros.h"
#include "rosserial_server/udp_socket_session.h"

using boost::asio::ip::udp;
using boost::asio::ip::address;

typedef boost::chrono::steady_clock time_source;

void controlThread(ros::Rate rate, ridgeback_base::RidgebackHardware* robot, controller_manager::ControllerManager* cm)
{
  time_source::time_point last_time = time_source::now();

  while (1)
  {
    // Calculate monotonic time elapsed
    time_source::time_point this_time = time_source::now();
    boost::chrono::duration<double> elapsed_duration = this_time - last_time;
    ros::Duration elapsed(elapsed_duration.count());
    last_time = this_time;

    if (robot->isActive())
    {
      robot->powerHasNotReset();
      robot->updateJointsFromHardware();
    }
    else
    {
      robot->configure();
    }

    robot->canSend();

    cm->update(ros::Time::now(), elapsed, robot->inReset());

    if (robot->isActive())
    {
      robot->command();
      robot->requestData();
    }
    else
    {
      robot->verify();
    }

    robot->canSend();
    rate.sleep();
  }
}

void canReadThread(ros::Rate rate, ridgeback_base::RidgebackHardware* robot)
{
  while (1)
  {
    robot->canRead();
    rate.sleep();
  }
}

int main(int argc, char* argv[])
{
  // Initialize ROS node.
  ros::init(argc, argv, "ridgeback_node");
  ros::NodeHandle nh, pnh("~");

  // Create the socket rosserial server in a background ASIO event loop.
  boost::asio::io_service io_service;

  new rosserial_server::UdpSocketSession(io_service, udp::endpoint(address::from_string("192.168.131.1"), 11411),
                                         udp::endpoint(address::from_string("192.168.131.2"), 11411));
  boost::thread(boost::bind(&boost::asio::io_service::run, &io_service));

  std::string canbus_dev;
  pnh.param<std::string>("canbus_dev", canbus_dev, "can0");
  puma_motor_driver::SocketCANGateway gateway(canbus_dev);

  ridgeback_base::RidgebackHardware ridgeback(nh, pnh, gateway);

  // Configure the CAN connection
  ridgeback.init();
  // Create a thread to start reading can messages.
  boost::thread canReadT(&canReadThread, ros::Rate(200), &ridgeback);

  // Background thread for the controls callback.
  ros::NodeHandle controller_nh("");
  controller_manager::ControllerManager cm(&ridgeback, controller_nh);
  boost::thread controlT(&controlThread, ros::Rate(25), &ridgeback, &cm);

  // Lighting control.
  ridgeback_base::RidgebackLighting lighting(&nh);

  // Create diagnostic updater, to update itself on the ROS thread.
  ridgeback_base::RidgebackDiagnosticUpdater ridgeback_diagnostic_updater;
  puma_motor_driver::PumaMotorDriverDiagnosticUpdater puma_motor_driver_diagnostic_updater;

  // Joint state publisher for passive front axle.
  ros::V_string passive_joint = boost::assign::list_of("front_rocker");
  ridgeback_base::PassiveJointPublisher passive_joint_publisher(nh, passive_joint, 50);

  // Cooling control for the fans.
  ridgeback_base::RidgebackCooling cooling(&nh);

  // Foreground ROS spinner for ROS callbacks, including rosserial, diagnostics
  ros::spin();

  return 0;
}
