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

#include "unix_test_harness.hpp"

#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace bno055_test
{

UnixTestHarness::UnixTestHarness()
{
  context = std::make_shared<rclcpp::Context>();
  context->init(0, {});

  fake_bno055 = std::make_unique<FakeBNO055>();

  rclcpp::NodeOptions node_options;
  node_options.context(context);
  node_options.parameter_overrides(
    std::vector<rclcpp::Parameter>
    {
      rclcpp::Parameter("port", fake_bno055->get_port_name()),
      rclcpp::Parameter("self_manage", true),
    });

  driver = std::make_unique<bno055_driver::BNO055Driver>("test_driver", node_options);

  rclcpp::executor::ExecutorArgs exe_args;
  exe_args.context = context;
  executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>(exe_args);
  executor->add_node(driver->get_node_base_interface());
}

}  // namespace bno055_test
