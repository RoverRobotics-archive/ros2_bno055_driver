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

#include "bno055_driver/bno055_uart.hpp"

#include <chrono>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>

namespace bno055_driver
{

BNO055UART::BNO055UART(const std::string & port)
: page_(255),
  port_(port, 115200)
{
  serial::Timeout timeout = serial::Timeout::simpleTimeout(200);
  port_.setTimeout(timeout);

  read_command_.message_type = BNO055MessageType::REGISTER_COMMAND;
  read_command_.command = BNO055RegisterCommand::READ;
  read_command_.address = BNO055Register::PAGE_ID;
  read_command_.length = 1;
  write_command_.message_type = BNO055MessageType::REGISTER_COMMAND;
  write_command_.command = BNO055RegisterCommand::WRITE;

  read();

  page_ = read_response_.data[0];
}

void BNO055UART::close()
{
  port_.close();
}

std::string BNO055UART::getPort()
{
  return port_.getPort();
}

bool BNO055UART::isOpen()
{
  return port_.isOpen();
}

void BNO055UART::read(uint8_t page)
{
  if (page != page_ && read_command_.address != BNO055Register::PAGE_ID) {
    // TODO(cottsay): Change pages automatically
    throw serial::SerialException("Page change required");
  }

  for (size_t this_write, index = 0; index < sizeof(read_command_); index += this_write) {
    this_write = write(&read_command_, index, 1);
    if (!this_write) {
      throw serial::SerialException("Timeout writing to device");
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  if (!read(&read_response_, 0, 1)) {
    throw serial::SerialException("Timeout reading from device");
  }

  if (read_response_.message_type == BNO055MessageType::RESPONSE_STATUS) {
    if (!read(&read_response_, 1, 1)) {
      throw serial::SerialException("Timeout reading from device");
    }

    std::stringstream ss;
    ss << "Register read failed with error code: 0x" << std::hex << std::setw(2) <<
      read_response_.response_status;
    throw serial::SerialException(ss.str().c_str());
  } else if (read_response_.message_type != BNO055MessageType::READ_RESPONSE) {
    std::stringstream ss;
    ss << "Invalid response from read command: 0x" << std::hex << std::setw(2) <<
      read_response_.message_type;
    throw serial::SerialException(ss.str().c_str());
  }

  if (!read(&read_response_, 1, 1)) {
    throw serial::SerialException("Timeout reading from device");
  }
  for (size_t this_read, remaining = read_response_.length; remaining > 0; remaining -= this_read) {
    this_read = read(&read_response_, 2, remaining);
    if (!this_read) {
      throw serial::SerialException("Timeout reading from device");
    }
  }

  if (read_command_.length != read_response_.length) {
    std::stringstream ss;
    ss << "Requested " << read_command_.length << " bytes from device, but got " <<
      read_response_.length;
    throw serial::SerialException(ss.str().c_str());
  }
}

template<typename T>
size_t BNO055UART::read(T * data, const size_t offset, size_t size)
{
  return port_.read(reinterpret_cast<uint8_t *>(data) + offset, size);
}

void BNO055UART::write(uint8_t page)
{
  if (page != page_ && write_command_.address != BNO055Register::PAGE_ID) {
    // TODO(cottsay): Change pages automtically
    throw serial::SerialException("Page change required");
  }

  // It has been observed that the BNO055 is not able to read bytes from the
  // UART interface as fast as they are sent. The problem is mitigated by
  // pausing between each byte.

  for (size_t this_write, index = 0; index < write_command_.length + 4u; index += this_write) {
    this_write = write(&write_command_, index, 1);
    if (!this_write) {
      throw serial::SerialException("Timeout writing to device");
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  if (!read(&write_response_, 0, 1)) {
    throw serial::SerialException("Timeout reading from device");
  }

  if (write_response_.message_type != BNO055MessageType::RESPONSE_STATUS) {
    std::stringstream ss;
    ss << "Invalid response from write command: 0x" << std::hex << std::setw(2) <<
      std::setfill('0') << write_response_.message_type;
    throw serial::SerialException(ss.str().c_str());
  }

  if (!read(&write_response_, 1, 1)) {
    throw serial::SerialException("Timeout reading from device");
  }

  if (write_response_.response_status != BNO055ResponseStatus::WRITE_SUCCESS) {
    std::stringstream ss;
    ss << "Register write failed with error code: 0x" << std::hex << std::setw(2) <<
      std::setfill('0') << write_response_.response_status;
    throw serial::SerialException(ss.str().c_str());
  }
}

template<typename T>
size_t BNO055UART::write(const T * data, const size_t offset, size_t size)
{
  return port_.write(reinterpret_cast<const uint8_t *>(data) + offset, size);
}

}  // namespace bno055_driver
