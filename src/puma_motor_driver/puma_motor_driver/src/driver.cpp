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

#include "puma_motor_driver/driver.h"
#include "puma_motor_driver/gateway.h"
#include "puma_motor_driver/message.h"
#include "puma_motor_msgs/MultiStatus.h"
#include "puma_motor_msgs/Status.h"
#include "puma_motor_msgs/MultiFeedback.h"
#include "puma_motor_msgs/Feedback.h"

#include <string>
#include <ros/ros.h>

namespace puma_motor_driver
{

Driver::Driver(Gateway& gateway, uint8_t device_number, std::string device_name)
  : gateway_(gateway), device_number_(device_number), device_name_(device_name),
    configured_(false), state_(0), control_mode_(puma_motor_msgs::Status::MODE_SPEED),
    gain_p_(1), gain_i_(0), gain_d_(0), encoder_cpr_(1), gear_ratio_(1)
  {
  }

void Driver::processMessage(const Message& received_msg)
{
  // If it's not our message, jump out.
  if (received_msg.getDeviceNumber() != device_number_) return;

  // If there's no data then this is a request message, jump out.
  if (received_msg.len == 0) return;

  StatusField* field = statusFieldForMessage(received_msg);

  // Wasn't a status message, return.
  if (!field) return;

  // Copy the received data and mark that field as received.
  memcpy(field->data, received_msg.data, received_msg.len);
  field->received = true;
}

void Driver::sendUint8(uint32_t id, uint8_t value)
{
  Message msg;
  msg.id = id;
  msg.len = 1;
  memcpy(msg.data, &value, msg.len);
  gateway_.queue(msg);
}

void Driver::sendUint16(uint32_t id, uint16_t value)
{
  Message msg;
  msg.id = id;
  msg.len = 2;
  memcpy(msg.data, &value, msg.len);
  gateway_.queue(msg);
}

void Driver::sendFixed8x8(uint32_t id, float value)
{
  Message msg;
  msg.id = id;
  msg.len = 2;
  int16_t output_value = static_cast<int16_t>(static_cast<float>(1<<8) * value);
  memcpy(msg.data, &output_value, msg.len);
  gateway_.queue(msg);
}

void Driver::sendFixed16x16(uint32_t id, double value)
{
  Message msg;
  msg.id = id;
  msg.len = 4;
  int32_t output_value = static_cast<int32_t>(static_cast<double>((1<<16) * value));
  memcpy(msg.data, &output_value, msg.len);
  gateway_.queue(msg);
}

bool Driver::verifyRaw16x16(uint8_t* received, double expected)
{
  uint8_t data[4];
  int32_t output_value = static_cast<int32_t>(static_cast<double>((1<<16) * expected));
  memcpy(data, &output_value, 4);
  for (uint8_t i = 0; i < 4; i++)
  {
    if (*received != data[i])
    {
      return false;
    }
    received++;
  }
  return true;
}

bool Driver::verifyRaw8x8(uint8_t* received, float expected)
{
  uint8_t data[2];
  int32_t output_value = static_cast<int32_t>(static_cast<float>((1<<8) * expected));
  memcpy(data, &output_value, 2);
  for (uint8_t i = 0; i < 2; i++)
  {
    if (*received != data[i])
    {
      return false;
    }
    received++;
  }
  return true;
}

void Driver::setEncoderCPR(uint16_t encoder_cpr)
{
  encoder_cpr_ = encoder_cpr;
}

void Driver::setGearRatio(float gear_ratio)
{
  gear_ratio_ = gear_ratio;
}

void Driver::commandDutyCycle(float cmd)
{
  sendFixed8x8((LM_API_VOLT_SET | device_number_), cmd);
}

void Driver::commandSpeed(double cmd)
{
  // Converting from rad/s to RPM through the gearbox.
  sendFixed16x16((LM_API_SPD_SET | device_number_), (cmd * ((60 * gear_ratio_) / (2 * M_PI))));
}

void Driver::verifyParams()
{
  switch (state_)
  {
    case 0:
      ROS_DEBUG("Dev: %i starting to verify parameters.", device_number_);
      state_++;
      break;
    case 1:
      if (lastPower() == 0)
      {
        state_++;
        ROS_DEBUG("Dev: %i cleared power flag.", device_number_);
      }
      else
      {
        gateway_.queue(Message(LM_API_STATUS_POWER | device_number_));
      }
      break;
    case 2:
      if (posEncoderRef() == LM_REF_ENCODER)
      {
        state_++;
        ROS_DEBUG("Dev: %i set position encoder reference.", device_number_);
      }
      else
      {
        gateway_.queue(Message(LM_API_POS_REF | device_number_));
      }
      break;
    case 3:
      if (spdEncoderRef() == LM_REF_QUAD_ENCODER)
      {
        state_++;
        ROS_DEBUG("Dev: %i set speed encoder reference.", device_number_);
      }
      else
      {
        gateway_.queue(Message(LM_API_SPD_REF | device_number_));
      }
      break;
    case 4:
      if (encoderCounts() == encoder_cpr_)
      {
        state_++;
        ROS_DEBUG("Dev: %i set encoder counts to %i.", device_number_, encoder_cpr_);
      }
      else
      {
        gateway_.queue(Message(LM_API_CFG_ENC_LINES | device_number_));
      }
      break;
    case 5:  // Need to enter a close loop mode to record encoder data.
      if (lastMode() == puma_motor_msgs::Status::MODE_SPEED)
      {
        state_++;
        ROS_DEBUG("Dev: %i entered a close-loop control mode.", device_number_);
      }
      else
      {
        gateway_.queue(Message(LM_API_STATUS_CMODE | device_number_));
      }
      break;
    case 6:
      if (lastMode() == control_mode_)
      {
        if (control_mode_ != puma_motor_msgs::Status::MODE_VOLTAGE)
        {
          state_++;
          ROS_DEBUG("Dev: %i  was set to a close loop control mode.", device_number_);
        }
        else
        {
          state_ = 200;
          ROS_DEBUG("Dev: %i was set to voltage control mode.", device_number_);
        }
      }
      break;
    case 7:
      if (verifyRaw16x16(getRawP(), gain_p_))
      {
        state_++;
        ROS_DEBUG("Dev: %i P gain constant was set to %f and %f was requested.", device_number_, getP(), gain_p_);
      }
      else
      {
        switch (control_mode_)
        {
          case puma_motor_msgs::Status::MODE_CURRENT:
            gateway_.queue(Message(LM_API_ICTRL_PC | device_number_));
            break;
          case puma_motor_msgs::Status::MODE_POSITION:
            gateway_.queue(Message(LM_API_POS_PC | device_number_));
            break;
          case puma_motor_msgs::Status::MODE_SPEED:
            gateway_.queue(Message(LM_API_SPD_PC | device_number_));
            break;
        }
      }
      break;
    case 8:
      if (verifyRaw16x16(getRawI(), gain_i_))
      {
        state_++;
        ROS_DEBUG("Dev: %i I gain constant was set to %f and %f was requested.", device_number_, getI(), gain_i_);
      }
      else
      {
        switch (control_mode_)
        {
          case puma_motor_msgs::Status::MODE_CURRENT:
            gateway_.queue(Message(LM_API_ICTRL_IC | device_number_));
            break;
          case puma_motor_msgs::Status::MODE_POSITION:
            gateway_.queue(Message(LM_API_POS_IC | device_number_));
            break;
          case puma_motor_msgs::Status::MODE_SPEED:
            gateway_.queue(Message(LM_API_SPD_IC | device_number_));
            break;
        }
      }
      break;
    case 9:
      if (verifyRaw16x16(getRawD(), gain_d_))
      {
        state_ = 200;
        ROS_DEBUG("Dev: %i D gain constant was set to %f and %f was requested.", device_number_, getD(), gain_d_);
      }
      else
      {
        switch (control_mode_)
        {
          case puma_motor_msgs::Status::MODE_CURRENT:
            gateway_.queue(Message(LM_API_ICTRL_DC | device_number_));
            break;
          case puma_motor_msgs::Status::MODE_POSITION:
            gateway_.queue(Message(LM_API_POS_DC | device_number_));
            break;
          case puma_motor_msgs::Status::MODE_SPEED:
            gateway_.queue(Message(LM_API_SPD_DC | device_number_));
            break;
        }
      }
      break;
  }
  if (state_ == 200)
  {
    ROS_INFO("Dev: %i all parameters verified.", device_number_);
    configured_ = true;
    state_ = 255;
  }
}

void Driver::configureParams()
{
  switch (state_)
  {
    case 1:
      sendUint8((LM_API_STATUS_POWER | device_number_), 1);
      break;
    case 2:
      sendUint8((LM_API_POS_REF | device_number_), LM_REF_ENCODER);
      break;
    case 3:
      sendUint8((LM_API_SPD_REF | device_number_), LM_REF_QUAD_ENCODER);
      break;
    case 4:
      // Set encoder CPR
      sendUint16((LM_API_CFG_ENC_LINES | device_number_), encoder_cpr_);
      break;
    case 5:  // Need to enter a close loop mode to record encoder data.
      gateway_.queue(Message(LM_API_SPD_EN | device_number_));
      break;
    case 6:
      switch (control_mode_)
      {
        case puma_motor_msgs::Status::MODE_VOLTAGE:
          gateway_.queue(Message(LM_API_VOLT_EN | device_number_));
          break;
        case puma_motor_msgs::Status::MODE_CURRENT:
          gateway_.queue(Message(LM_API_ICTRL_EN | device_number_));
          break;
        case puma_motor_msgs::Status::MODE_POSITION:
          gateway_.queue(Message(LM_API_POS_EN | device_number_));
          break;
        case puma_motor_msgs::Status::MODE_SPEED:
          gateway_.queue(Message(LM_API_SPD_EN | device_number_));
          break;
      }
      break;
    case 7:
      // Set P
      switch (control_mode_)
      {
        case puma_motor_msgs::Status::MODE_CURRENT:
          sendFixed16x16((LM_API_ICTRL_PC  | device_number_), gain_p_);
          break;
        case puma_motor_msgs::Status::MODE_POSITION:
          sendFixed16x16((LM_API_POS_PC  | device_number_), gain_p_);
          break;
        case puma_motor_msgs::Status::MODE_SPEED:
          sendFixed16x16((LM_API_SPD_PC  | device_number_), gain_p_);
          break;
      }
      break;
    case 8:
      // Set I
      switch (control_mode_)
      {
        case puma_motor_msgs::Status::MODE_CURRENT:
          sendFixed16x16((LM_API_ICTRL_IC  | device_number_), gain_i_);
          break;
        case puma_motor_msgs::Status::MODE_POSITION:
          sendFixed16x16((LM_API_POS_IC  | device_number_), gain_i_);
          break;
        case puma_motor_msgs::Status::MODE_SPEED:
          sendFixed16x16((LM_API_SPD_IC  | device_number_), gain_i_);
          break;
      }
      break;
    case 9:
      // Set D
      switch (control_mode_)
      {
        case puma_motor_msgs::Status::MODE_CURRENT:
          sendFixed16x16((LM_API_ICTRL_DC  | device_number_), gain_d_);
          break;
        case puma_motor_msgs::Status::MODE_POSITION:
          sendFixed16x16((LM_API_POS_DC  | device_number_), gain_d_);
          break;
        case puma_motor_msgs::Status::MODE_SPEED:
          sendFixed16x16((LM_API_SPD_DC  | device_number_), gain_d_);
          break;
      }
      break;
  }
}

bool Driver::isConfigured()
{
  return configured_;
}

void Driver::setGains(double p, double i, double d)
{
  gain_p_ = p;
  gain_i_ = i;
  gain_d_ = d;

  if (configured_)
  {
    updateGains();
  }
}

void Driver::setMode(uint8_t mode)
{
  if (mode == puma_motor_msgs::Status::MODE_VOLTAGE)
  {
    control_mode_ = mode;
    ROS_INFO("Dev: %i mode set to voltage control.", device_number_);
    if (configured_)
    {
      resetConfiguration();
    }
  }
  else
  {
    ROS_ERROR("Close loop modes need PID gains.");
  }
}

void Driver::setMode(uint8_t mode, double p, double i, double d)
{
  if (mode == puma_motor_msgs::Status::MODE_VOLTAGE)
  {
    control_mode_ = mode;
    ROS_WARN("Dev: %i mode set to voltage control but PID gains are not needed.", device_number_);
    if (configured_)
    {
      resetConfiguration();
    }
  }
  else
  {
    control_mode_ = mode;
    if (configured_)
    {
      resetConfiguration();
    }
    setGains(p, i, d);
    ROS_INFO("Dev: %i mode set to a closed-loop control with PID gains of P:%f, I:%f and D:%f.",
      device_number_, gain_p_, gain_i_, gain_d_);
  }
}

void Driver::clearStatusCache()
{
  // Set it all to zero, which will in part clear
  // the boolean flags to be false.
  memset(status_fields_, 0, sizeof(status_fields_));
}

void Driver::requestStatusMessages()
{
  gateway_.queue(Message(LM_API_STATUS_POWER   | device_number_));
}

void Driver::requestFeedbackMessages()
{
  gateway_.queue(Message(LM_API_STATUS_VOLTOUT | device_number_));
  gateway_.queue(Message(LM_API_STATUS_CURRENT | device_number_));
  gateway_.queue(Message(LM_API_STATUS_POS     | device_number_));
  gateway_.queue(Message(LM_API_STATUS_SPD     | device_number_));
  gateway_.queue(Message(LM_API_SPD_SET        | device_number_));
}
void Driver::requestFeedbackDutyCycle()
{
  gateway_.queue(Message(LM_API_STATUS_VOLTOUT | device_number_));
}

void Driver::requestFeedbackCurrent()
{
  gateway_.queue(Message(LM_API_STATUS_CURRENT | device_number_));
}

void Driver::requestFeedbackPosition()
{
  gateway_.queue(Message(LM_API_STATUS_POS | device_number_));
}

void Driver::requestFeedbackSpeed()
{
  gateway_.queue(Message(LM_API_STATUS_SPD | device_number_));
}

void Driver::requestFeedbackPowerState()
{
  gateway_.queue(Message(LM_API_STATUS_POWER | device_number_));
}

void Driver::requestFeedbackSetpoint()
{
  switch (control_mode_)
  {
    case puma_motor_msgs::Status::MODE_CURRENT:
      gateway_.queue(Message(LM_API_ICTRL_SET | device_number_));
      break;
    case puma_motor_msgs::Status::MODE_POSITION:
      gateway_.queue(Message(LM_API_POS_SET | device_number_));
      break;
    case puma_motor_msgs::Status::MODE_SPEED:
      gateway_.queue(Message(LM_API_SPD_SET | device_number_));
      break;
    case puma_motor_msgs::Status::MODE_VOLTAGE:
      gateway_.queue(Message(LM_API_VOLT_SET | device_number_));
      break;
  };
}

void Driver::resetConfiguration()
{
  configured_ = false;
  state_ = 0;
}

void Driver::updateGains()
{
  configured_ = false;
  state_ = 7;
}

float Driver::lastDutyCycle()
{
  StatusField* field = statusFieldForMessage(Message(LM_API_STATUS_VOLTOUT));
  return (field->interpretFixed8x8() / 128.0);
}

float Driver::lastBusVoltage()
{
  StatusField* field = statusFieldForMessage(Message(LM_API_STATUS_VOLTBUS));
  return field->interpretFixed8x8();
}

float Driver::lastCurrent()
{
  StatusField* field = statusFieldForMessage(Message(LM_API_STATUS_CURRENT));
  return field->interpretFixed8x8();
}

double Driver::lastPosition()
{
  StatusField* field = statusFieldForMessage(Message(LM_API_STATUS_POS));
  return (field->interpretFixed16x16() * ((2 * M_PI) / gear_ratio_));  // Convert rev to rad
}

double Driver::lastSpeed()
{
  StatusField* field = statusFieldForMessage(Message(LM_API_STATUS_SPD));
  return (field->interpretFixed16x16() * ((2 * M_PI) / (gear_ratio_ * 60)));  // Convert RPM to rad/s
}

uint8_t Driver::lastFault()
{
  StatusField* field = statusFieldForMessage(Message(LM_API_STATUS_FAULT));
  return field->data[0];
}

uint8_t Driver::lastPower()
{
  StatusField* field = statusFieldForMessage(Message(LM_API_STATUS_POWER));
  return field->data[0];
}

uint8_t Driver::lastMode()
{
  StatusField* field = statusFieldForMessage(Message(LM_API_STATUS_CMODE));
  return field->data[0];
}

float Driver::lastOutVoltage()
{
  StatusField* field = statusFieldForMessage(Message(LM_API_STATUS_VOUT));
  return field->interpretFixed8x8();
}

float Driver::lastTemperature()
{
  StatusField* field = statusFieldForMessage(Message(LM_API_STATUS_TEMP));
  return field->interpretFixed8x8();
}

float Driver::lastAnalogInput()
{
  StatusField* field = statusFieldForMessage(Message(CPR_API_STATUS_ANALOG));
  return field->interpretFixed8x8();
}

double Driver::lastSetpoint()
{
  switch (control_mode_)
  {
    case puma_motor_msgs::Status::MODE_CURRENT:
      return statusCurrentGet();
      break;
    case puma_motor_msgs::Status::MODE_POSITION:
      return statusPositionGet();
      break;
    case puma_motor_msgs::Status::MODE_SPEED:
      return statusSpeedGet();
      break;
    case puma_motor_msgs::Status::MODE_VOLTAGE:
      return statusDutyCycleGet();
      break;
    default:
      return 0;
      break;
  }
}
double Driver::statusSpeedGet()
{
  StatusField* field = statusFieldForMessage(Message(LM_API_SPD_SET));
  return (field->interpretFixed16x16() * ((2 * M_PI) / (gear_ratio_ * 60)));  // Convert RPM to rad/s
}

float Driver::statusDutyCycleGet()
{
  StatusField* field = statusFieldForMessage(Message(LM_API_VOLT_SET));
  return (field->interpretFixed8x8() / 128.0);
}

float Driver::statusCurrentGet()
{
  StatusField* field = statusFieldForMessage(Message(LM_API_ICTRL_SET));
  return field->interpretFixed8x8();
}
double Driver::statusPositionGet()
{
  StatusField* field = statusFieldForMessage(Message(LM_API_POS_SET));
  return (field->interpretFixed16x16() * (( 2 * M_PI) / gear_ratio_));  // Convert rev to rad
}

uint8_t Driver::posEncoderRef()
{
  StatusField* field = statusFieldForMessage(Message(LM_API_POS_REF));
  return field->data[0];
}

uint8_t Driver::spdEncoderRef()
{
  StatusField* field = statusFieldForMessage(Message(LM_API_SPD_REF));
  return field->data[0];
}

uint16_t Driver::encoderCounts()
{
  StatusField* field = statusFieldForMessage(Message(LM_API_CFG_ENC_LINES));
  return (static_cast<uint16_t>(field->data[0]) | static_cast<uint16_t>(field->data[1] << 8));
}

double Driver::getP()
{
  StatusField* field;
  switch (control_mode_)
  {
    case puma_motor_msgs::Status::MODE_CURRENT:
      field = statusFieldForMessage(Message(LM_API_ICTRL_PC));
      break;
    case puma_motor_msgs::Status::MODE_POSITION:
      field = statusFieldForMessage(Message(LM_API_POS_PC));
      break;
    case puma_motor_msgs::Status::MODE_SPEED:
      field = statusFieldForMessage(Message(LM_API_SPD_PC));
      break;
  };
  return field->interpretFixed16x16();
}

double Driver::getI()
{
  StatusField* field;
  switch (control_mode_)
  {
    case puma_motor_msgs::Status::MODE_CURRENT:
      field = statusFieldForMessage(Message(LM_API_ICTRL_IC));
      break;
    case puma_motor_msgs::Status::MODE_POSITION:
      field = statusFieldForMessage(Message(LM_API_POS_IC));
      break;
    case puma_motor_msgs::Status::MODE_SPEED:
      field = statusFieldForMessage(Message(LM_API_SPD_IC));
      break;
  };
  return field->interpretFixed16x16();
}

double Driver::getD()
{
  StatusField* field;
  switch (control_mode_)
  {
    case puma_motor_msgs::Status::MODE_CURRENT:
      field = statusFieldForMessage(Message(LM_API_ICTRL_DC));
      break;
    case puma_motor_msgs::Status::MODE_POSITION:
      field = statusFieldForMessage(Message(LM_API_POS_DC));
      break;
    case puma_motor_msgs::Status::MODE_SPEED:
      field = statusFieldForMessage(Message(LM_API_SPD_DC));
      break;
  };
  return field->interpretFixed16x16();
}

uint8_t* Driver::getRawP()
{
  StatusField* field;
  switch (control_mode_)
  {
    case puma_motor_msgs::Status::MODE_CURRENT:
      field = statusFieldForMessage(Message(LM_API_ICTRL_PC));
      break;
    case puma_motor_msgs::Status::MODE_POSITION:
      field = statusFieldForMessage(Message(LM_API_POS_PC));
      break;
    case puma_motor_msgs::Status::MODE_SPEED:
      field = statusFieldForMessage(Message(LM_API_SPD_PC));
      break;
  };
  return field->data;
}

uint8_t* Driver::getRawI()
{
  StatusField* field;
  switch (control_mode_)
  {
    case puma_motor_msgs::Status::MODE_CURRENT:
      field = statusFieldForMessage(Message(LM_API_ICTRL_IC));
      break;
    case puma_motor_msgs::Status::MODE_POSITION:
      field = statusFieldForMessage(Message(LM_API_POS_IC));
      break;
    case puma_motor_msgs::Status::MODE_SPEED:
      field = statusFieldForMessage(Message(LM_API_SPD_IC));
      break;
  };
  return field->data;
}

uint8_t* Driver::getRawD()
{
  StatusField* field;
  switch (control_mode_)
  {
    case puma_motor_msgs::Status::MODE_CURRENT:
      field = statusFieldForMessage(Message(LM_API_ICTRL_DC));
      break;
    case puma_motor_msgs::Status::MODE_POSITION:
      field = statusFieldForMessage(Message(LM_API_POS_DC));
      break;
    case puma_motor_msgs::Status::MODE_SPEED:
      field = statusFieldForMessage(Message(LM_API_SPD_DC));
      break;
  };
  return field->data;
}
Driver::StatusField* Driver::statusFieldForMessage(const Message& msg)
{
  // If it's not a STATUS message, there is no status field box to return.
  // if ((msg.getApi() & CAN_MSGID_API_M) != CAN_API_MC_STATUS)
  // {
  //   return NULL;
  // }

  uint32_t status_field_index = (msg.getApi() & CAN_MSGID_API_ID_M) >> CAN_MSGID_API_S;
  return &status_fields_[status_field_index];
}

}  // namespace puma_motor_driver
