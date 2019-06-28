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

#ifndef FAKE_BNO055_HPP_
#define FAKE_BNO055_HPP_

#include <stdint.h>

#include <memory>
#include <string>
#include <thread>

#define FAKE_BNO055_NUM_PAGES 2
#define FAKE_BNO055_PAGE_SIZE 256

namespace bno055_test
{

class FakeBNO055
{
public:
  FakeBNO055();
  ~FakeBNO055();
  void initialize();
  void close();
  std::string get_port_name() const;

protected:
  void process_read_request(const uint8_t page, const uint8_t address, const uint8_t size) const;
  void process_write_request(
    const uint8_t page, const uint8_t address, const uint8_t size, const uint8_t * data);
  void spin();

  static const uint8_t REGISTER_DEFAULTS[FAKE_BNO055_NUM_PAGES][FAKE_BNO055_PAGE_SIZE];

private:
  uint8_t registers[FAKE_BNO055_NUM_PAGES][FAKE_BNO055_PAGE_SIZE];

  std::unique_ptr<std::thread> spin_thread;

  char port_name[256];
  int master_fd;
  int slave_fd;
};

}  // namespace bno055_test

#endif  // FAKE_BNO055_HPP_
