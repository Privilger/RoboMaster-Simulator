/**
Software License Agreement (BSD)

\file      mecanum_plugin.cpp
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

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/ode/ODESurfaceParams.hh>
#include <gazebo/physics/physics.hh>
#include <ros/console.h>

#include <string>


namespace gazebo
{

class MecanumPlugin : public ModelPlugin
{
public:
  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

protected:
  virtual void GazeboUpdate();

private:
  event::ConnectionPtr update_connection_;
  physics::ModelPtr model_;
  physics::LinkPtr wheel_link_;
  physics::LinkPtr fixed_link_;
  double roller_angle_;
  double roller_friction_;
  double roller_skid_friction_;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(MecanumPlugin);

void MecanumPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  // Store the model
  model_ = _parent;

  std::string link_name;

  if (_sdf->HasElement("wheelLinkName"))
  {
    link_name = _sdf->Get<std::string>("wheelLinkName");
    wheel_link_ = model_->GetLink(link_name);
    if (!wheel_link_)
    {
      ROS_FATAL_STREAM("Wheel link [" << link_name << "] not found!");
      return;
    }
  }
  else
  {
    ROS_FATAL("The mecanum plugin requires a `wheelLinkName` parameter.");
    return;
  }

  if (_sdf->HasElement("fixedLinkName"))
  {
    link_name = _sdf->Get<std::string>("fixedLinkName");
    fixed_link_ = model_->GetLink(link_name);
    if (!fixed_link_)
    {
      ROS_FATAL_STREAM("Fixed link [" << link_name << "] not found!");
      return;
    }
  }
  else
  {
    ROS_FATAL("The mecanum plugin requires a `fixedLinkName` parameter.");
    return;
  }

  if (_sdf->HasElement("rollerAngle"))
  {
    roller_angle_ = _sdf->Get<double>("rollerAngle");
  }
  else
  {
    roller_angle_ = M_PI / 4;
  }

  if (_sdf->HasElement("rollerFriction"))
  {
    roller_friction_ = _sdf->Get<double>("rollerFriction");
  }
  else
  {
    roller_friction_ = 100;
  }

  if (_sdf->HasElement("rollerSkidFriction"))
  {
    roller_skid_friction_ = _sdf->Get<double>("rollerSkidFriction");
  }
  else
  {
    roller_skid_friction_ = 100000;
  }

  ROS_INFO_STREAM("Mecanum plugin initialized for " << wheel_link_->GetName() <<
                  ", referenced to " << fixed_link_->GetName() << ", with a roller " <<
                  "angle of " << roller_angle_ << " radians.");

  // Register update event handler
  this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&MecanumPlugin::GazeboUpdate, this));
}

void MecanumPlugin::GazeboUpdate()
{
  math::Pose wheel_pose = wheel_link_->GetWorldCoGPose();
  math::Pose fixed_pose = fixed_link_->GetWorldCoGPose();
  math::Quaternion wheel_orientation = wheel_pose.CoordRotationSub(fixed_pose.rot);
  double wheel_angle = wheel_orientation.GetPitch();
  ROS_DEBUG_STREAM(wheel_link_->GetName() << " angle is " << wheel_angle << " radians.");

  // TODO: Understand better what the deal is with multiple collisions on a link.
  unsigned int collision_index = 0;
  physics::SurfaceParamsPtr surface = wheel_link_->GetCollision(collision_index)->GetSurface();

  // TODO: Check that the physics engine is ODE to avoid a segfault here.
  physics::ODESurfaceParams* ode_surface = dynamic_cast<physics::ODESurfaceParams*>(surface.get());
  physics::FrictionPyramid& fric(ode_surface->frictionPyramid);

  // TODO: Parameterize these.
  fric.SetMuPrimary(roller_skid_friction_);
  fric.SetMuSecondary(roller_friction_);

  // TODO: Investigate replacing this manual trigonometry with Pose::rot::RotateVector. Doing this
  // would also make it easier to support wheels which rotate about an axis other than Y.
  fric.direction1.x = cos(roller_angle_) * cos(wheel_angle);
  fric.direction1.y = sin(roller_angle_);
  fric.direction1.z = cos(roller_angle_) * sin(wheel_angle);
}

}  // namespace gazebo
