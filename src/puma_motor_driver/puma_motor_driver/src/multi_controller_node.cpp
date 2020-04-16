/**
Software License Agreement (BSD)

\authors   Mike Purvis <mpurvis@clearpathrobotics.com>
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
#include <vector>

#include "boost/foreach.hpp"
#include "boost/scoped_ptr.hpp"
#include "boost/shared_ptr.hpp"
#include "serial/serial.h"

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "puma_motor_driver/driver.h"
#include "puma_motor_driver/serial_gateway.h"
#include "puma_motor_driver/socketcan_gateway.h"
#include "puma_motor_driver/multi_driver_node.h"
#include "puma_motor_driver/diagnostic_updater.h"
#include "puma_motor_msgs/MultiStatus.h"
#include "puma_motor_msgs/Status.h"
#include "puma_motor_msgs/MultiFeedback.h"
#include "puma_motor_msgs/Feedback.h"


class MultiControllerNode
{
public:
  MultiControllerNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private,
                      puma_motor_driver::Gateway& gateway) :
    nh_(nh),
    nh_private_(nh_private),
    gateway_(gateway),
    active_(false),
    status_count_(0),
    desired_mode_(puma_motor_msgs::Status::MODE_SPEED)
  {
    drivers_.push_back(puma_motor_driver::Driver(gateway_, 3, "fl"));
    drivers_.push_back(puma_motor_driver::Driver(gateway_, 5, "fr"));
    drivers_.push_back(puma_motor_driver::Driver(gateway_, 2, "rl"));
    drivers_.push_back(puma_motor_driver::Driver(gateway_, 4, "rr"));

    cmd_sub_ = nh_.subscribe("cmd", 1, &MultiControllerNode::cmdCallback, this);

    nh_private_.param<double>("gear_ratio", gear_ratio_, 79.0);
    nh_private_.param<int>("encoder_cpr", encoder_cpr_, 1024);
    nh_private_.param<int>("frequency", freq_, 25);


    BOOST_FOREACH(puma_motor_driver::Driver& driver, drivers_)
    {
      driver.clearStatusCache();
      driver.setEncoderCPR(encoder_cpr_);
      driver.setGearRatio(gear_ratio_);
      driver.setMode(desired_mode_, 0.1, 0.01, 0.0);
    }

    multi_driver_node_.reset(new puma_motor_driver::MultiDriverNode(nh_, drivers_));
  }


  void cmdCallback(const sensor_msgs::JointStateConstPtr& cmd_msg)
  {
    if (active_)
    {
      BOOST_FOREACH(puma_motor_driver::Driver& driver, drivers_)
      {
        for (int i = 0; i < cmd_msg->name.size(); i++)
        {
          if (driver.deviceName() == cmd_msg->name[i])
          {
            if (desired_mode_ == puma_motor_msgs::Status::MODE_VOLTAGE)
            {
              driver.commandDutyCycle(cmd_msg->velocity[i]);
            }
            else if (desired_mode_ == puma_motor_msgs::Status::MODE_SPEED)
            {
              driver.commandSpeed(cmd_msg->velocity[i] * 6.28);
            }
          }
        }
      }
    }
  }


  bool connectIfNotConnected()
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

  void run()
  {
    ros::Rate rate(freq_);

    while (ros::ok())
    {
      if (!connectIfNotConnected())
      {
        ros::Duration(1.0).sleep();
        continue;
      }

      if (active_)
      {
        // Checks to see if power flag has been reset for each driver
        BOOST_FOREACH(puma_motor_driver::Driver& driver, drivers_)
        {
          if (driver.lastPower() != 0)
          {
            active_ = false;
            multi_driver_node_->activePublishers(active_);
            ROS_WARN("Power reset detected on device ID %d, will reconfigure all drivers.", driver.deviceNumber());
            BOOST_FOREACH(puma_motor_driver::Driver& driver, drivers_)
            {
              driver.resetConfiguration();
            }
          }
        }
        // Queue data requests for the drivers in order to assemble an amalgamated status message.
        BOOST_FOREACH(puma_motor_driver::Driver& driver, drivers_)
        {
          driver.requestStatusMessages();
          driver.requestFeedbackSetpoint();
        }
      }
      else
      {
        // Set parameters for each driver.
        BOOST_FOREACH(puma_motor_driver::Driver& driver, drivers_)
        {
          driver.configureParams();
        }
      }
      gateway_.sendAllQueued();
      // Process ROS callbacks, which will queue command messages to the drivers.
      ros::spinOnce();
      gateway_.sendAllQueued();
      // ros::Duration(0.005).sleep();


      // Process all received messages through the connected driver instances.
      puma_motor_driver::Message recv_msg;
      while (gateway_.recv(&recv_msg))
      {
        BOOST_FOREACH(puma_motor_driver::Driver& driver, drivers_)
        {
          driver.processMessage(recv_msg);
        }
      }

      // Check parameters of each driver instance.
      if (!active_)
      {
        BOOST_FOREACH(puma_motor_driver::Driver& driver, drivers_)
        {
          driver.verifyParams();
        }
      }

      // Verify that the all drivers are configured.
      if ( drivers_[0].isConfigured() == true
        && drivers_[1].isConfigured() == true
        && drivers_[2].isConfigured() == true
        && drivers_[3].isConfigured() == true
        && active_ == false)
      {
        active_ = true;
        multi_driver_node_->activePublishers(active_);
        ROS_INFO("All contollers active.");
      }
      // Send the broadcast heartbeat message.
      // gateway_.heartbeat();
      status_count_++;
      rate.sleep();
    }
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  puma_motor_driver::Gateway& gateway_;
  std::vector<puma_motor_driver::Driver> drivers_;

  int freq_;
  int encoder_cpr_;
  double gear_ratio_;
  uint8_t status_count_;
  uint8_t desired_mode_;
  bool active_;

  ros::Subscriber cmd_sub_;
  boost::shared_ptr<puma_motor_driver::MultiDriverNode> multi_driver_node_;
};


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "puma_multi_controller_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  std::string serial_port;
  std::string canbus_dev;

  boost::scoped_ptr<puma_motor_driver::Gateway> gateway;

  if (nh_private.getParam("canbus_dev", canbus_dev))
  {
    gateway.reset(new puma_motor_driver::SocketCANGateway(canbus_dev));
  }
  else if (nh_private.getParam("serial_port", serial_port))
  {
    serial::Serial serial;
    serial.setPort(serial_port);
    gateway.reset(new puma_motor_driver::SerialGateway(serial));
  }
  else
  {
    ROS_FATAL("No communication method given.");
    return 1;
  }

  puma_motor_driver::PumaMotorDriverDiagnosticUpdater puma_motor_driver_diagnostic_updater;
  MultiControllerNode n(nh, nh_private, *gateway);
  n.run();
}
