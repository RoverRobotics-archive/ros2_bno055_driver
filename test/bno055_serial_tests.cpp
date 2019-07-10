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

#include <gtest/gtest.h>

#include "rclcpp/rclcpp.hpp"
#include "unix_test_harness.hpp"

TEST(bno055_serial_tests, basic_configuration)
{
  bno055_test::UnixTestHarness test;

  test.driver->register_on_activate(
    [executor = test.executor](const rclcpp_lifecycle::State &) {
      executor->cancel();
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    });

  test.executor->spin();
}

int main(int argc, char ** argv)
{
  rclcpp::install_signal_handlers();

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
