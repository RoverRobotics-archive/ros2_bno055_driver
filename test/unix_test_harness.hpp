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

#ifndef UNIX_TEST_HARNESS_HPP_
#define UNIX_TEST_HARNESS_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "fake_bno055.hpp"
#include "bno055_driver/bno055_driver.hpp"

namespace bno055_test
{

class UnixTestHarness
{
public:
  UnixTestHarness();

  rclcpp::Context::SharedPtr context;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor;

  bno055_driver::BNO055Driver::UniquePtr driver;
  std::unique_ptr<FakeBNO055> fake_bno055;
};

}  // namespace bno055_test

#endif  // UNIX_TEST_HARNESS_HPP_
