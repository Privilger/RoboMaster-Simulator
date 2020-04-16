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
#ifndef PUMA_MOTOR_DRIVER_GATEWAY_H
#define PUMA_MOTOR_DRIVER_GATEWAY_H

#include "puma_motor_driver/can_proto.h"
#include "puma_motor_driver/message.h"

#include <stdint.h>


namespace puma_motor_driver
{

class Gateway
{
public:
  virtual bool connect() = 0;
  virtual bool isConnected() = 0;

  /**
   * Queue specified message to be sent on the bus.
   */
  virtual void queue(const Message& msg) = 0;

  /**
   * Send the queued messages on the bus.
   */
  virtual bool sendAllQueued() = 0;

  /**
   * Receive the next available message from the bus, blocking for
   * timeout_millis if nonzero.
   *
   * \return True if a message was returned false if timeout occurred.
   */
  virtual bool recv(Message* msg) = 0;
};

}  // namespace puma_motor_driver

#endif  // PUMA_MOTOR_DRIVER_GATEWAY_H
