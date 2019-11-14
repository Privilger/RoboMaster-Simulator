/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, PAL Robotics, S.L.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the PAL Robotics nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*
 * Author: Enrique Fernández
 */

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>

#include <nav_msgs/Odometry.h>
#include <tf/tfMessage.h>

#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

#include <mecanum_drive_controller/odometry.h>
#include <mecanum_drive_controller/speed_limiter.h>

namespace mecanum_drive_controller
{

/**
 * This class makes some assumptions on the model of the robot:
 *  - the rotation axes of wheels are collinear
 *  - the wheels are identical in radius
 * Additional assumptions to not duplicate information readily available in the URDF:
 *  - the wheels have the same parent frame
 *  - a wheel collision geometry is a cylinder in the urdf
 *  - a wheel joint frame center's vertical projection on the floor must lie within the contact patch
 */
class MecanumDriveController
    : public controller_interface::Controller<hardware_interface::VelocityJointInterface>
{
public:
  MecanumDriveController();

  /**
   * \brief Initialize controller
   * \param hw            Velocity joint interface for the wheels
   * \param root_nh       Node handle at root namespace
   * \param controller_nh Node handle inside the controller namespace
   */
  bool init(hardware_interface::VelocityJointInterface* hw,
            ros::NodeHandle& root_nh,
            ros::NodeHandle &controller_nh);

  /**
   * \brief Updates controller, i.e. computes the odometry and sets the new velocity commands
   * \param time   Current time
   * \param period Time since the last called to update
   */
  void update(const ros::Time& time, const ros::Duration& period);

  /**
   * \brief Starts controller
   * \param time Current time
   */
  void starting(const ros::Time& time);

  /**
   * \brief Stops controller
   * \param time Current time
   */
  void stopping(const ros::Time& time);

private:
  std::string name_;

  /// Odometry related:
  ros::Duration publish_period_;
  ros::Time last_state_publish_time_;
  bool open_loop_;

  /// Hardware handles:
  hardware_interface::JointHandle wheel0_jointHandle_;
  hardware_interface::JointHandle wheel1_jointHandle_;
  hardware_interface::JointHandle wheel2_jointHandle_;
  hardware_interface::JointHandle wheel3_jointHandle_;

  /// Velocity command related:
  struct Commands
  {
    double linX;
    double linY;
    double ang;
    ros::Time stamp;

    Commands() : linX(0.0), linY(0.0), ang(0.0), stamp(0.0) {}
  };
  realtime_tools::RealtimeBuffer<Commands> command_;
  Commands command_struct_;
  ros::Subscriber sub_command_;

  /// Odometry related:
  boost::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry> > odom_pub_;
  boost::shared_ptr<realtime_tools::RealtimePublisher<tf::tfMessage> > tf_odom_pub_;
  Odometry odometry_;
  geometry_msgs::TransformStamped odom_frame_;

  /// Wheel radius (assuming it's the same for the left and right wheels):
  bool use_realigned_roller_joints_;
  double wheels_k_; // wheels geometric param used in mecanum wheels' ik
  double wheels_radius_;
  double wheel_separation_x_;
  double wheel_separation_y_;

  /// Timeout to consider cmd_vel commands old:
  double cmd_vel_timeout_;

  /// Frame to use for the robot base:
  std::string base_frame_id_;

  /// Whether to publish odometry to tf or not:
  bool enable_odom_tf_;

  /// Number of wheel joints:
  size_t wheel_joints_size_;

  // Speed limiters:
  Commands last_cmd_;
  SpeedLimiter limiter_linX_;
  SpeedLimiter limiter_linY_;
  SpeedLimiter limiter_ang_;

private:
  /**
   * \brief Brakes the wheels, i.e. sets the velocity to 0
   */
  void brake();

  /**
   * \brief Velocity command callback
   * \param command Velocity command message (twist)
   */
  void cmdVelCallback(const geometry_msgs::Twist& command);

  /**
   * \brief Sets odometry parameters from the URDF, i.e. the wheel radius and separation
   * \param root_nh Root node handle
   * \param wheel0_name Name of wheel0 joint
   * \param wheel1_name Name of wheel1 joint
   * \param wheel2_name Name of wheel2 joint
   * \param wheel3_name Name of wheel3 joint
   */
  bool setWheelParamsFromUrdf(ros::NodeHandle& root_nh,
                             ros::NodeHandle &controller_nh,
                             const std::string& wheel0_name,
                             const std::string& wheel1_name,
                             const std::string& wheel2_name,
                             const std::string& wheel3_name);

  /**
   * \brief Get the radius of a given wheel
   * \param       model         urdf model used
   * \param       wheel_link    link of the wheel from which to get the radius
   * \param[out]  wheels_radius radius of the wheel read from the urdf
   */
  bool getWheelRadius(const urdf::ModelInterfaceSharedPtr model, const urdf::LinkConstSharedPtr& wheel_link, double& wheel_radius);

  /**
   * \brief Sets the odometry publishing fields
   * \param root_nh Root node handle
   * \param controller_nh Node handle inside the controller namespace
   */
  void setupRtPublishersMsg(ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);

};

PLUGINLIB_EXPORT_CLASS(mecanum_drive_controller::MecanumDriveController, controller_interface::ControllerBase)

} // namespace mecanum_drive_controller
