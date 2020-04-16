/**
Software License Agreement (BSD)

\authors   Tony Baltovski <tbaltovsi@clearpathrobotics.com>
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

#include <gtest/gtest.h>
#include <iostream>
#include "puma_motor_driver/driver.h"
#include "puma_motor_driver/message.h"

TEST(TestFixedPoint16x16, test16x16interpret)
{
  double cases[7] =
  {
    0.0,
    1.0,
    -1.0,
    -0.00001,
    5000.00001,
    32000.000025,
    -32000.000025
  };

  for (int i = 0; i < 7; i++)
  {
    puma_motor_driver::Driver::StatusField status;
    int32_t input = cases[i] * static_cast<double>(1<<16);
    memcpy(&status.data, &input, 4);
    EXPECT_NEAR(cases[i], status.interpretFixed16x16(), 1.0 / static_cast<double>(1<<16));
  }
}

TEST(TestFixedPoint8x8, test8x8interpret)
{
  float cases[7] =
  {
    0.0,
    1.0,
    -1.0,
    -0.00001,
    127.001,
    -127.002,
    123.42678
  };

  for (int i = 0; i < 7; i++)
  {
    puma_motor_driver::Driver::StatusField status;
    int16_t input = cases[i] * static_cast<float>(1<<8);
    memcpy(&status.data, &input, 2);
    EXPECT_NEAR(cases[i], status.interpretFixed8x8(), 1.0 / static_cast<float>(1<<8));
  }
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
