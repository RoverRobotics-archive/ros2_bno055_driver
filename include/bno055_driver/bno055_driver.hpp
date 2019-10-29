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

#ifndef BNO055_DRIVER__BNO055_DRIVER_HPP_
#define BNO055_DRIVER__BNO055_DRIVER_HPP_

#include "bno055_driver/bno055_reg.hpp"
#include "bno055_driver/bno055_uart.hpp"

#include <stdint.h>

#include <memory>
#include <string>

#include "lifecycle_msgs/srv/change_state.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/temperature.hpp"

#include "rclcpp/client.hpp"
#include "rclcpp/rate.hpp"

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "bno055_driver/visibility_control.hpp"

namespace bno055_driver
{

class BNO055Driver : public rclcpp_lifecycle::LifecycleNode
{
public:
  BNO055Driver();

  BNO055_DRIVER_PUBLIC
  explicit BNO055Driver(const rclcpp::NodeOptions & options);

  explicit BNO055Driver(
    const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State &);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_error(const rclcpp_lifecycle::State &);

protected:
  void publish();
  void publish_diagnostics();

private:
  std::shared_ptr<BNO055UART> port_;
  bool use_magnetometer_;

  sensor_msgs::msg::Imu::SharedPtr imu_msg_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  sensor_msgs::msg::MagneticField::SharedPtr mag_msg_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
  sensor_msgs::msg::Temperature::SharedPtr tmp_msg_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Temperature>::SharedPtr tmp_pub_;
  diagnostic_msgs::msg::DiagnosticArray::SharedPtr diag_msg_;
  rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;

  rclcpp::TimerBase::SharedPtr imu_timer_;
  rclcpp::TimerBase::SharedPtr diag_timer_;

  rclcpp::GenericRate<std::chrono::system_clock> connection_rate_;

  lifecycle_msgs::srv::ChangeState::Request::SharedPtr change_state_request_;
  rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr change_state_client_;
  rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedFuture change_state_future_;
};

}  // namespace bno055_driver

#endif  // BNO055_DRIVER__BNO055_DRIVER_HPP_
