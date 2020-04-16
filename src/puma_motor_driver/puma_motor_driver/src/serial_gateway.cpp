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

#include "puma_motor_driver/serial_gateway.h"
#include "ros/ros.h"

namespace puma_motor_driver
{

SerialGateway::SerialGateway(serial::Serial& serial) : serial_(serial),
  write_buffer_index_(0), read_buffer_index_(0), read_buffer_len_(0)
{
  serial::Timeout to(serial::Timeout::simpleTimeout(50));
  serial_.setTimeout(to);
  serial_.setBaudrate(115200);
}

bool SerialGateway::connect()
{
  if (!serial_.isOpen())
  {
    try
    {
      serial_.open();
    }
    catch (std::exception& e)
    {
      ROS_ERROR("Exception thrown trying to open %s: %s", serial_.getPort().c_str(), e.what());
      return false;
    }
    return true;
  }
  return false;
}

bool SerialGateway::isConnected()
{
  return serial_.isOpen();
}

void SerialGateway::queue(const Message& msg)
{
  ROS_DEBUG_NAMED("serial", "Queuing ID 0x%08x, data (%d)", msg.id, msg.len);

  queue('\xff');
  queue(4 + msg.len);

  encodeAndQueue(reinterpret_cast<uint8_t*>(msg.id), 4);
  encodeAndQueue(msg.data, msg.len);
}

bool SerialGateway::recv(Message* msg)
{
  int bytes_dropped = 0;

  // Look for header byte.
  while (1)
  {
    uint8_t header_byte;
    if (!read(&header_byte))
    {
      ROS_DEBUG_NAMED("serial", "Header byte read timed out. No more messages for now.");
      return false;
    }

    if (header_byte != 0xff)
    {
      bytes_dropped++;
      continue;
    }

    ROS_WARN_COND_NAMED(bytes_dropped > 0, "serial", "Discarded %d bytes between messages.", bytes_dropped);
    break;
  }

  uint8_t len_byte;
  if (!read(&len_byte))
  {
    ROS_WARN_NAMED("serial", "Len byte read timed out.");
    return false;
  }
  if (len_byte < 4 || len_byte > 12)
  {
    ROS_WARN_NAMED("serial", "Illegal length byte value, dropping message.");
    return false;
  }

  // CAN ID in little endian.
  if (!readAndDecode(reinterpret_cast<uint8_t*>(msg->id), 4))
  {
    ROS_WARN_NAMED("serial", "Problem reading ID, dropping message.");
    return false;
  }

  // CAN data.
  msg->len = len_byte - 4;
  if (!readAndDecode(msg->data, msg->len))
  {
    ROS_WARN_NAMED("serial", "Problem reading %d data bytes for ID 0x%08x, dropping message.", msg->len, msg->id);
    return false;
  }

  ROS_DEBUG_NAMED("serial", "Received ID 0x%08x, data (%d)", msg->id, msg->len);
  return true;
}

void SerialGateway::queue(uint8_t ch)
{
  if (write_buffer_index_ < sizeof(write_buffer_))
  {
    write_buffer_[write_buffer_index_] = ch;
    write_buffer_index_++;
  }
}

bool SerialGateway::read(uint8_t* ch)
{
  // If buffer is exhausted, attempt to request more from the hardware.
  if (read_buffer_index_ >= read_buffer_len_)
  {
    read_buffer_index_ = 0;
    read_buffer_len_ = serial_.read(read_buffer_, sizeof(read_buffer_));
    ROS_DEBUG_NAMED("serial", "Filled read buffer with %d bytes.", read_buffer_len_);

    if (read_buffer_len_ < 1)
    {
      // Read operation returned nothing, therefore there is no character
      // to return here.
      return false;
    }
  }

  *ch = read_buffer_[read_buffer_index_];
  read_buffer_index_++;
  return true;
}

bool SerialGateway::sendAllQueued()
{
  try
  {
    uint8_t written = serial_.write(write_buffer_, write_buffer_index_);

    if (written < write_buffer_index_)
    {
      ROS_WARN_STREAM("Write to serial port timed out. Tried to write " <<
          write_buffer_index_ << " bytes, instead wrote only " << written << ". Remainder dropped.");
      write_buffer_index_ = 0;
      return false;
    }
    ROS_DEBUG_NAMED("serial", "Wrote %d bytes to serial port.", written);
  }
  catch (std::exception& e)
  {
    ROS_ERROR("Exception thrown trying to write to %s: %s", serial_.getPort().c_str(), e.what());
    serial_.close();
  }

  write_buffer_index_ = 0;
  return true;
}

void SerialGateway::encodeAndQueue(const uint8_t* data, uint8_t len)
{
  for (uint8_t index = 0; index < len; index++)
  {
    switch (data[index])
    {
      case 0xff:
        queue('\xfe');
        queue('\xfe');
        break;
      case 0xfe:
        queue('\xfe');
        queue('\xfd');
        break;
      default:
        queue(data[index]);
    }
  }
}

bool SerialGateway::readAndDecode(uint8_t* data, uint8_t len)
{
  for (uint8_t index = 0; index < len; index++)
  {
    uint8_t undecoded_byte;
    if (!read(&undecoded_byte))
    {
      ROS_WARN_NAMED("serial", "Read of undecoded byte timed out.");
      return false;
    }

    switch (undecoded_byte)
    {
      case 0xff:
        ROS_WARN_NAMED("serial", "Unexpected start-of-message character.");
        return false;
      case 0xfe:
        uint8_t second_byte;
        if (!read(&second_byte))
        {
          ROS_WARN_NAMED("serial", "Read of second byte in decoded pair timed out.");
          return false;
        }
        switch (second_byte)
        {
          case 0xfe:
            data[index] = '\xff';
            break;
          case 0xfd:
            data[index] = '\xfe';
            break;
          default:
            ROS_WARN_NAMED("serial", "Unexpected second byte in encoded sequence.");
            return false;
        }
        break;
      default:
        data[index] = undecoded_byte;
    }
  }
  return true;
}

}  // namespace puma_motor_driver
