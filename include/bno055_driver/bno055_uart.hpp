// Copyright 2019 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef BNO055_DRIVER__BNO055_UART_HPP_
#define BNO055_DRIVER__BNO055_UART_HPP_

#include "serial/serial.h"

#include "bno055_driver/bno055_reg.hpp"
#include "bno055_driver/visibility_control.h"

namespace bno055_driver
{

struct BNO055ReadCommand
{
  BNO055MessageType message_type;
  BNO055RegisterCommand command;
  BNO055Register address;
  uint8_t length;
} __attribute__ ((__packed__));

struct BNO055ReadResponse
{
  BNO055MessageType message_type;
  union
  {
    uint8_t length;
    BNO055ResponseStatus response_status;
  };
  uint8_t data[128];
} __attribute__ ((__packed__));

struct BNO055WriteCommand
{
  BNO055MessageType message_type;
  BNO055RegisterCommand command;
  BNO055Register address;
  uint8_t length;
  uint8_t data[128];
} __attribute__ ((__packed__));

struct BNO055WriteResponse
{
  BNO055MessageType message_type;
  BNO055ResponseStatus response_status;
} __attribute__ ((__packed__));

class BNO055UART
{
public:
  BNO055UART(const std::string &port);
  void close();
  std::string getPort();
  bool isOpen();
  void read(uint8_t page = 0);
  void write(uint8_t page = 0);

  BNO055ReadCommand read_command_;
  BNO055ReadResponse read_response_;
  BNO055WriteCommand write_command_;
  BNO055WriteResponse write_response_;
protected:
  void setPage(uint8_t page);
private:
  uint8_t page_;
  serial::Serial port_;
};

}  // namespace bno055_driver

#endif  // BNO055_DRIVER__BNO055_DRIVER_HPP_

