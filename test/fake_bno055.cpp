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

#include "fake_bno055.hpp"

#include <string.h>
#include <unistd.h>

#if defined(__linux__)
#include <pty.h>
#else
#include <util.h>
#endif

#include <memory>
#include <string>
#include <system_error>

#include "bno055_driver/bno055_reg.hpp"

namespace bno055_test
{

const uint8_t FakeBNO055::REGISTER_DEFAULTS[FAKE_BNO055_NUM_PAGES][FAKE_BNO055_PAGE_SIZE]
{
  {
    0xA0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  },
};

FakeBNO055::FakeBNO055()
: port_name(""), master_fd(-1), slave_fd(-1)
{
  initialize();

  if (::openpty(&master_fd, &slave_fd, port_name, NULL, NULL) == -1) {
    throw std::system_error(errno, std::generic_category());
  }

  spin_thread = std::make_unique<std::thread>(&FakeBNO055::spin, this);
}

FakeBNO055::~FakeBNO055()
{
  close();
  spin_thread->join();
}

void FakeBNO055::initialize()
{
  memcpy(&registers, &REGISTER_DEFAULTS, sizeof(registers));
}

void FakeBNO055::close()
{
  ::close(slave_fd);
  slave_fd = -1;

  ::close(master_fd);
  master_fd = -1;
}

std::string FakeBNO055::get_port_name() const
{
  return std::string(port_name);
}

void FakeBNO055::process_read_request(
  const uint8_t page, const uint8_t address, const uint8_t size) const
{
  if (page > FAKE_BNO055_NUM_PAGES || (size_t)address + size > FAKE_BNO055_PAGE_SIZE) {
    printf("invalid read request of %d bytes at %d:%d\n", size, page, address);
    return;
  }

  uint8_t buffer[] =
  {
    bno055_driver::BNO055MessageType::READ_RESPONSE,
    size,
  };

  if (::write(master_fd, buffer, sizeof(buffer)) != sizeof(buffer)) {
    throw std::system_error(errno, std::generic_category());
  }
  if (::write(master_fd, &registers[page][address], size) != size) {
    throw std::system_error(errno, std::generic_category());
  }
}

void FakeBNO055::process_write_request(
  const uint8_t page, const uint8_t address, const uint8_t size, const uint8_t * data)
{
  if (page > FAKE_BNO055_NUM_PAGES || (size_t)address + size > FAKE_BNO055_PAGE_SIZE) {
    printf("invalid read request of %d bytes at %d:%d\n", size, page, address);
    return;
  }

  memcpy(&registers[page][address], data, size);

  uint8_t buffer[] =
  {
    bno055_driver::BNO055MessageType::RESPONSE_STATUS,
    bno055_driver::BNO055ResponseStatus::WRITE_SUCCESS,
  };

  if (::write(master_fd, buffer, sizeof(buffer)) != sizeof(buffer)) {
    throw std::system_error(errno, std::generic_category());
  }
}

void FakeBNO055::spin()
{
  uint8_t buffer[FAKE_BNO055_PAGE_SIZE];
  int ret, pos = 0, target = 4;

  struct timeval timeout
  {
    0,
    0,
  };

  fd_set set;
  FD_ZERO(&set);

  while (master_fd >= 0) {
    timeout.tv_usec = 100000;
    FD_SET(master_fd, &set);

    ret = ::select(master_fd + 1, &set, NULL, NULL, &timeout);
    if (ret < 0) {
      printf("select failed: %d\n", errno);
      return;
    } else if (ret == 0) {
      pos = 0;
      target = 4;
      continue;
    }

    ret = ::read(master_fd, &buffer[pos], 1);
    if (ret <= 0) {
      printf("read failed: %d\n", ret == 0 ? ENODATA : errno);
      return;
    }

    pos += ret;
    if (pos < target) {
      continue;
    }

    if (buffer[0] != bno055_driver::BNO055MessageType::REGISTER_COMMAND) {
      printf("invalid message type: %d\n", buffer[0]);
      return;
    }

    switch (buffer[1]) {
      case bno055_driver::BNO055RegisterCommand::READ:
        process_read_request(
          registers[0][bno055_driver::BNO055Register::PAGE_ID], buffer[2], buffer[3]);
        break;
      case bno055_driver::BNO055RegisterCommand::WRITE:
        if (target == 4) {
          target += buffer[3];
          continue;
        }
        process_write_request(
          registers[0][bno055_driver::BNO055Register::PAGE_ID], buffer[2], buffer[3], &buffer[4]);
        break;
      default:
        printf("invalid register command: %d\n", buffer[1]);
        return;
    }

    pos = 0;
    target = 4;
  }
}

}  // namespace bno055_test
