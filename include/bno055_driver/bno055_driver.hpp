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

#include <stdint.h>

#include "lifecycle_msgs/srv/change_state.hpp"

#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/temperature.hpp"

#include "rclcpp/client.hpp"

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "diagnostic_updater/diagnostic_updater.hpp"

#include "bno055_driver/bno055_reg.hpp"
#include "bno055_driver/bno055_uart.hpp"
#include "bno055_driver/visibility_control.h"

namespace bno055_driver
{

class BNO055Driver : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit BNO055Driver(const std::string & node_name);

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
  void publishDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &wrapper);

private:
  std::shared_ptr<BNO055UART> port_;
  bool use_magnetometer_;

  std::shared_ptr<sensor_msgs::msg::Imu> imu_msg_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Imu>> imu_pub_;
  std::shared_ptr<sensor_msgs::msg::MagneticField> mag_msg_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::MagneticField>> mag_pub_;
  std::shared_ptr<sensor_msgs::msg::Temperature> tmp_msg_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Temperature>> tmp_pub_;

  rclcpp::TimerBase::SharedPtr imu_timer_;

  diagnostic_updater::Updater diagnostic_updater_;

  std::shared_ptr<lifecycle_msgs::srv::ChangeState::Request> change_state_request_;
  std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>> change_state_client_;
  rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedFuture change_state_future_;
};

}  // namespace bno055_driver

#endif  // BNO055_DRIVER__BNO055_DRIVER_HPP_
