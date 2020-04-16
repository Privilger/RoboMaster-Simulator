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
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include "puma_motor_driver/socketcan_gateway.h"
#include "ros/ros.h"

namespace puma_motor_driver
{

SocketCANGateway::SocketCANGateway(std::string canbus_dev):
  canbus_dev_(canbus_dev),
  is_connected_(false),
  write_frames_index_(0)
{
}

bool SocketCANGateway::connect()
{
  if ((socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
  {
    ROS_ERROR("Error while opening socket");
    return false;
  }

  struct ifreq ifr;

  snprintf (ifr.ifr_name, sizeof(canbus_dev_.c_str()), "%s", canbus_dev_.c_str());

  if (ioctl(socket_, SIOCGIFINDEX, &ifr) < 0)
  {
    close(socket_);
    ROS_ERROR("Error while trying to control device");
    return false;
  }

  struct sockaddr_can addr;
  addr.can_family  = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  ROS_DEBUG("%s at index %d", canbus_dev_.c_str(), ifr.ifr_ifindex);

  if (bind(socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0)
  {
    ROS_ERROR("Error in socket bind");
    return false;
  }

  struct timeval tv;
  tv.tv_sec = 0;
  tv.tv_usec = 1;  // microseconds

  setsockopt(socket_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  ROS_INFO("Opened Socket CAN on %s", canbus_dev_.c_str());
  is_connected_ = true;

  return is_connected_;
}


bool SocketCANGateway::isConnected()
{
  return is_connected_;
}


bool SocketCANGateway::recv(Message* msg)
{
  can_frame read_frame;

  int bytes = read(socket_, &read_frame, sizeof(struct can_frame));
  if (bytes == sizeof(struct can_frame))
  {
    ROS_DEBUG("Recieved ID 0x%08x, data (%d)", (read_frame.can_id& CAN_EFF_MASK), read_frame.can_dlc);
    msgToFrame(msg, &read_frame);
    return true;
  }
  else
  {
    if (bytes < 0)
    {
      if (errno == EAGAIN)
      {
        ROS_DEBUG("No more frames");
      }
      else
      {
        ROS_ERROR("Error reading from socketcan: %d", errno);
      }
    }
    else
    {
      ROS_ERROR("Socketcan read() returned unexpected size.");
    }
    return false;
  }
}

void SocketCANGateway::queue(const Message& msg)
{
  ROS_DEBUG("Queuing ID 0x%08x, data (%d)", msg.id, msg.len);
  write_frames_[write_frames_index_].can_id = msg.id | CAN_EFF_FLAG;
  write_frames_[write_frames_index_].can_dlc = msg.len;

  for (int i = 0; i < msg.len; i++)
  {
    write_frames_[write_frames_index_].data[i] = msg.data[i];
  }
  write_frames_index_++;
}

bool SocketCANGateway::sendAllQueued()
{
  for (int i = 0; i < write_frames_index_; i++)
  {
    ROS_DEBUG("Writing ID 0x%08x, data (%d)", write_frames_[i].can_id, write_frames_[i].can_dlc);
    int bytes = write(socket_, &write_frames_[i], sizeof(struct can_frame));
  }
  write_frames_index_ = 0;
  return true;
}

void SocketCANGateway::msgToFrame(Message* msg, can_frame* frame)
{
  msg->id = frame->can_id & CAN_EFF_MASK;
  msg->len = frame->can_dlc;
  for (int i = 0; i < msg->len; i++)
  {
    msg->data[i] = frame->data[i];
  }
}

}  // namespace puma_motor_driver
