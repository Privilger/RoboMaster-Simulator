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

#ifndef PUMA_MOTOR_DRIVER_SOCKETCAN_GATEWAY_H
#define PUMA_MOTOR_DRIVER_SOCKETCAN_GATEWAY_H


#include <string>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <net/if.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include "puma_motor_driver/gateway.h"
#include "puma_motor_driver/message.h"

namespace puma_motor_driver
{

class SocketCANGateway : public Gateway
{
public:
  explicit SocketCANGateway(std::string canbus_dev);

  virtual bool connect();
  virtual bool isConnected();

  virtual bool recv(Message* msg);
  virtual void queue(const Message& msg);
  virtual bool sendAllQueued();
  void msgToFrame(Message* msg, can_frame* frame);

private:
  int socket_;
  std::string canbus_dev_;  // CAN interface ID
  bool is_connected_;

  can_frame write_frames_[1024];
  int write_frames_index_;
};

}  // namespace puma_motor_driver

#endif  // PUMA_MOTOR_DRIVER_SOCKETCAN_GATEWAY_H
