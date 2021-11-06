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

#include "bno055_driver/bno055_driver.hpp"
#include "bno055_driver/bno055_scales.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp_components/register_node_macro.hpp"

#define bytes_to_ushort(addr) (*reinterpret_cast<uint16_t *>(addr))
#define bytes_to_short(addr) (*reinterpret_cast<int16_t *>(addr))

RCLCPP_COMPONENTS_REGISTER_NODE(bno055_driver::BNO055Driver)

namespace bno055_driver
{

BNO055Driver::BNO055Driver()
: BNO055Driver("bno055_driver")
{
}

BNO055Driver::BNO055Driver(const rclcpp::NodeOptions & options)
: BNO055Driver("bno055_driver", options)
{
}

BNO055Driver::BNO055Driver(const std::string & node_name, const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode(node_name, options),
  connection_rate_(4.0)
{
  RCLCPP_INFO(get_logger(), "%s is called.", __func__);

  declare_parameter("diagnostic_period", rclcpp::ParameterValue((rcl_duration_value_t)1));
  declare_parameter("frame_id");
  declare_parameter("frequency", rclcpp::ParameterValue(30.0f));
  declare_parameter("port", rclcpp::ParameterValue("/dev/ttyUSB0"));
  declare_parameter("self_manage", rclcpp::ParameterValue(false));

  declare_parameter("use_magnetometer", true);

  declare_parameter("angular_velocity_stdev");
  declare_parameter("linear_acceleration_stdev");
  declare_parameter("magnetic_field_stdev");
  declare_parameter("orientation_stdev");

  declare_parameter("calibration.accelerometer_offset");
  declare_parameter("calibration.gyroscope_offset");
  declare_parameter("calibration.magnetometer_offset");
  declare_parameter("calibration.accelerometer_radius");
  declare_parameter("calibration.magnetometer_radius");

  /**
   * Set SI by default after ROS REP 103
   */
  declare_parameter<std::string>("unit.angular_velocity", "rps");
  declare_parameter<std::string>("unit.linear_acceleration", "mps2");
  declare_parameter<std::string>("unit.magnetic_field", "T");
  declare_parameter<std::string>("unit.temperature", "C");

  if (get_parameter("self_manage").get_value<bool>()) {
    change_state_request_ = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    change_state_client_ = this->create_client<lifecycle_msgs::srv::ChangeState>(
      std::string(get_name()) + "/change_state",
      rmw_qos_profile_services_default,
      nullptr);

    RCLCPP_INFO(get_logger(), "Self-transitioning to INACTIVE");
    change_state_request_->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE;
    change_state_request_->transition.label = "";
    change_state_future_ = change_state_client_->async_send_request(change_state_request_);
  }
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
BNO055Driver::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "%s is called.", __func__);

  // Rate-limit our connection attempts
  connection_rate_.sleep();

  rclcpp::Parameter port_name = get_parameter("port");
  port_ = std::make_shared<BNO055UART>(port_name.get_value<std::string>());
  RCLCPP_DEBUG(get_logger(), "Opened connection to '%s'.", port_->getPort().c_str());

  // Switch to register page 0
  port_->write_command_.length = 1;
  port_->write_command_.address = BNO055Register::PAGE_ID;
  port_->write_command_.data[0] = 0;
  port_->write();

  // Read the chip ID
  port_->read_command_.length = 1;
  port_->read_command_.address = BNO055Register::CHIP_ID;
  port_->read();
  if (port_->read_response_.data[0] != 0xA0) {
    RCLCPP_DEBUG(get_logger(), "Invalid BNO055 chip ID: 0x%02X", port_->read_response_.data[0]);
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  // Switch to config mode
  port_->write_command_.address = BNO055Register::OPR_MODE;
  port_->write_command_.data[0] = BNO055OperationMode::CONFIG;
  port_->write();

  // Set the units
  uint8_t unit_config = 0b00000110;
  rclcpp::Parameter angular_velocity_param;
  if (get_parameter("unit.angular_velocity", angular_velocity_param) &&
    angular_velocity_param.get_type() == rclcpp::PARAMETER_STRING) {
    if (angular_velocity_param.as_string() == "rps") {
      RCLCPP_INFO(get_logger(), "Setting unit angular_velocity to rad/s");
      unit_config |= 0b00000010;
      vel_scale_ = scale::vel_to_rps;
    } else if (angular_velocity_param.as_string() == "dps") {
      RCLCPP_INFO(get_logger(), "Setting unit angular_velocity to deg/s");
      unit_config &= ~0b00000010;
      vel_scale_ = scale::vel_to_dps;
    } else {
      RCLCPP_ERROR(get_logger(), "Invalid value for unit angular_velocity");
    }
  }

  rclcpp::Parameter linear_acceleration_param;
  if (get_parameter("unit.linear_acceleration", linear_acceleration_param) &&
    linear_acceleration_param.get_type() == rclcpp::PARAMETER_STRING) {
    if (linear_acceleration_param.as_string() == "mps2") {
      RCLCPP_INFO(get_logger(), "Setting unit linear_acceleration to m/s2");
      unit_config &= ~0b00000001;
      lacc_scale_ = scale::acc_to_mps2;
    } else if (linear_acceleration_param.as_string() == "mg") {
      RCLCPP_INFO(get_logger(), "Setting unit linear_acceleration to mg");
      unit_config |= 0b00000001;
      lacc_scale_ = scale::acc_to_mg;
    } else if (linear_acceleration_param.as_string() == "g") {
      RCLCPP_INFO(get_logger(), "Setting unit linear_acceleration to g");
      unit_config |= 0b00000001;
      lacc_scale_ = scale::acc_to_g;
    } else {
      RCLCPP_ERROR(get_logger(), "Invalid value for unit linear_acceleration");
    }
  }

  rclcpp::Parameter magnetic_field_param;
  if (get_parameter("unit.magnetic_field", magnetic_field_param) &&
    magnetic_field_param.get_type() == rclcpp::PARAMETER_STRING) {
    if (magnetic_field_param.as_string() == "T") {
      RCLCPP_INFO(get_logger(), "Setting unit magnetic_field to T");
      mag_scale_ = scale::mag_to_t;
    } else if (magnetic_field_param.as_string() == "uT") {
      RCLCPP_INFO(get_logger(), "Setting unit magnetic_field to uT");
      mag_scale_ = scale::mag_to_ut;
    } else {
      RCLCPP_ERROR(get_logger(), "Invalid value for unit magnetic_field");
    }
  }

  rclcpp::Parameter temperature_param;
  if (get_parameter("unit.temperature", temperature_param) &&
    temperature_param.get_type() == rclcpp::PARAMETER_STRING) {
    if (temperature_param.as_string() == "C") {
      RCLCPP_INFO(get_logger(), "Setting unit temperature to C");
      unit_config &= ~0b00010000;
      temp_scale_ = scale::temp_to_c;
    } else if (temperature_param.as_string() == "F") {
      RCLCPP_INFO(get_logger(), "Setting unit temperature to F");
      unit_config |= 0b00010000;
      temp_scale_ = scale::temp_to_f;
    } else {
      RCLCPP_ERROR(get_logger(), "Invalid value for unit temperature");
    }
  }

  port_->write_command_.address = BNO055Register::UNIT_SEL;
  port_->write_command_.data[0] = unit_config;
  port_->write();

  // Set power mode
  port_->write_command_.address = BNO055Register::PWR_MODE;
  port_->write_command_.data[0] = BNO055PowerMode::NORMAL;
  port_->write();

  // Select internal oscillator
  port_->write_command_.address = BNO055Register::SYS_TRIGGER;
  port_->write_command_.data[0] = 0x00;
  port_->write();

  use_magnetometer_ = get_parameter("use_magnetometer").get_value<bool>();

  // If given, set the calibration
  rclcpp::Parameter accelerometer_offset_param;
  if (get_parameter("calibration.accelerometer_offset", accelerometer_offset_param)) {
    const std::vector<int64_t> accelerometer_offset =
      accelerometer_offset_param.get_value<std::vector<int64_t>>();

    if (accelerometer_offset.size() != 3) {
      RCLCPP_ERROR(get_logger(), "Invalid value for calibration.accelerometer_offset");
    } else {
      RCLCPP_INFO(get_logger(), "Setting accelerometer calibration offsets");

      port_->write_command_.address = BNO055Register::ACC_OFFSET_X_LSB;
      port_->write_command_.length = 6;
      port_->write_command_.data[0] = accelerometer_offset[0] & 0xFF;
      port_->write_command_.data[1] = accelerometer_offset[0] >> 8 & 0xFF;
      port_->write_command_.data[2] = accelerometer_offset[1] & 0xFF;
      port_->write_command_.data[3] = accelerometer_offset[1] >> 8 & 0xFF;
      port_->write_command_.data[4] = accelerometer_offset[2] & 0xFF;
      port_->write_command_.data[5] = accelerometer_offset[2] >> 8 & 0xFF;
      port_->write();
    }
  }

  rclcpp::Parameter gyroscope_offset_param;
  if (get_parameter("calibration.gyroscope_offset", gyroscope_offset_param)) {
    const std::vector<int64_t> gyroscope_offset =
      gyroscope_offset_param.get_value<std::vector<int64_t>>();

    if (gyroscope_offset.size() != 3) {
      RCLCPP_ERROR(get_logger(), "Invalid value for calibration.gyroscope_offset");
    } else {
      RCLCPP_INFO(get_logger(), "Setting gyroscope calibration offsets");

      port_->write_command_.address = BNO055Register::GYR_OFFSET_X_LSB;
      port_->write_command_.length = 6;
      port_->write_command_.data[0] = gyroscope_offset[0] & 0xFF;
      port_->write_command_.data[1] = gyroscope_offset[0] >> 8 & 0xFF;
      port_->write_command_.data[2] = gyroscope_offset[1] & 0xFF;
      port_->write_command_.data[3] = gyroscope_offset[1] >> 8 & 0xFF;
      port_->write_command_.data[4] = gyroscope_offset[2] & 0xFF;
      port_->write_command_.data[5] = gyroscope_offset[2] >> 8 & 0xFF;
      port_->write();
    }
  }

  rclcpp::Parameter magnetometer_offset_param;
  if (get_parameter("calibration.magnetometer_offset", magnetometer_offset_param)) {
    const std::vector<int64_t> magnetometer_offset =
      magnetometer_offset_param.get_value<std::vector<int64_t>>();

    if (magnetometer_offset.size() != 3) {
      RCLCPP_ERROR(get_logger(), "Invalid value for calibration.magnetometer_offset");
    } else {
      RCLCPP_INFO(get_logger(), "Setting magnetometer calibration offsets");

      port_->write_command_.address = BNO055Register::MAG_OFFSET_X_LSB;
      port_->write_command_.length = 6;
      port_->write_command_.data[0] = magnetometer_offset[0] & 0xFF;
      port_->write_command_.data[1] = (magnetometer_offset[0] >> 8) & 0xFF;
      port_->write_command_.data[2] = magnetometer_offset[1] & 0xFF;
      port_->write_command_.data[3] = (magnetometer_offset[1] >> 8) & 0xFF;
      port_->write_command_.data[4] = magnetometer_offset[2] & 0xFF;
      port_->write_command_.data[5] = (magnetometer_offset[2] >> 8) & 0xFF;
      port_->write();
    }
  }

  rclcpp::Parameter accelerometer_radius_param;
  if (get_parameter("calibration.accelerometer_radius", accelerometer_radius_param)) {
    const int64_t accelerometer_radius = accelerometer_radius_param.get_value<int64_t>();

    RCLCPP_INFO(get_logger(), "Setting accelerometer calibration radius");

    port_->write_command_.address = BNO055Register::ACC_RADIUS_LSB;
    port_->write_command_.length = 2;
    port_->write_command_.data[0] = accelerometer_radius & 0xFF;
    port_->write_command_.data[1] = (accelerometer_radius >> 8) & 0xFF;
    port_->write();
  }

  rclcpp::Parameter magnetometer_radius_param;
  if (get_parameter("calibration.magnetometer_radius", magnetometer_radius_param)) {
    const int64_t magnetometer_radius = magnetometer_radius_param.get_value<int64_t>();

    RCLCPP_INFO(get_logger(), "Setting magnetometer calibration radius");

    port_->write_command_.address = BNO055Register::MAG_RADIUS_LSB;
    port_->write_command_.length = 2;
    port_->write_command_.data[0] = magnetometer_radius & 0xFF;
    port_->write_command_.data[1] = (magnetometer_radius >> 8) & 0xFF;
    port_->write();
  }

  imu_msg_ = std::make_shared<sensor_msgs::msg::Imu>();
  imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
    "imu/data",
    rclcpp::SensorDataQoS());
  mag_msg_ = std::make_shared<sensor_msgs::msg::MagneticField>();
  mag_pub_ = this->create_publisher<sensor_msgs::msg::MagneticField>(
    "imu/mag",
    rclcpp::SensorDataQoS());
  tmp_msg_ = std::make_shared<sensor_msgs::msg::Temperature>();
  tmp_pub_ = this->create_publisher<sensor_msgs::msg::Temperature>(
    "imu/temp",
    rclcpp::SensorDataQoS());
  diag_msg_ = std::make_shared<diagnostic_msgs::msg::DiagnosticArray>();
  diag_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/diagnostics",
    rclcpp::SystemDefaultsQoS());
  diag_msg_->status.resize(1);
  diag_msg_->status[0].name = "BNO055 Status";
  diag_msg_->status[0].hardware_id = std::string("BNO055 IMU on ") + port_->getPort();
  diag_msg_->status[0].values.reserve(7);

  rclcpp::Parameter frame_id_param;
  if (get_parameter("frame_id", frame_id_param)) {
    const std::string frame_id = frame_id_param.get_value<std::string>();
    imu_msg_->header.frame_id = frame_id;
    mag_msg_->header.frame_id = frame_id;
    tmp_msg_->header.frame_id = frame_id;
    diag_msg_->header.frame_id = frame_id;
  }

  rclcpp::Parameter angular_velocity_stdev_param;
  if (get_parameter("angular_velocity_stdev", angular_velocity_stdev_param)) {
    const double angular_velocity_stdev = angular_velocity_stdev_param.get_value<double>();
    const double angular_velocity_variance = angular_velocity_stdev * angular_velocity_stdev;
    imu_msg_->angular_velocity_covariance[0] = angular_velocity_variance;
    imu_msg_->angular_velocity_covariance[4] = angular_velocity_variance;
    imu_msg_->angular_velocity_covariance[8] = angular_velocity_variance;
  }

  rclcpp::Parameter linear_acceleration_stdev_param;
  if (get_parameter("linear_acceleration_stdev", linear_acceleration_stdev_param)) {
    const double linear_acceleration_stdev = linear_acceleration_stdev_param.get_value<double>();
    const double linear_acceleration_variance =
      linear_acceleration_stdev * linear_acceleration_stdev;
    imu_msg_->linear_acceleration_covariance[0] = linear_acceleration_variance;
    imu_msg_->linear_acceleration_covariance[4] = linear_acceleration_variance;
    imu_msg_->linear_acceleration_covariance[8] = linear_acceleration_variance;
  }

  rclcpp::Parameter magnetic_field_stdev_param;
  if (get_parameter("magnetic_field_stdev", magnetic_field_stdev_param)) {
    const double magnetic_field_stdev = magnetic_field_stdev_param.get_value<double>();
    const double magnetic_field_variance = magnetic_field_stdev * magnetic_field_stdev;
    mag_msg_->magnetic_field_covariance[0] = magnetic_field_variance;
    mag_msg_->magnetic_field_covariance[4] = magnetic_field_variance;
    mag_msg_->magnetic_field_covariance[8] = magnetic_field_variance;
  }

  rclcpp::Parameter orientation_stdev_param;
  if (get_parameter("orientation_stdev", orientation_stdev_param)) {
    const double orientation_stdev = orientation_stdev_param.get_value<double>();
    const double orientation_variance = orientation_stdev * orientation_stdev;
    imu_msg_->orientation_covariance[0] = orientation_variance;
    imu_msg_->orientation_covariance[4] = orientation_variance;
    imu_msg_->orientation_covariance[8] = orientation_variance;
  }

  rclcpp::Parameter frequency = get_parameter("frequency");
  imu_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / frequency.get_value<float>())),
    std::bind(&BNO055Driver::publish, this));

  rclcpp::Parameter diagnostic_period = get_parameter("diagnostic_period");
  diag_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<rcl_duration_value_t>(
        diagnostic_period.get_value<rcl_duration_value_t>())),
    std::bind(&BNO055Driver::publish_diagnostics, this));

  if (get_parameter("self_manage").get_value<bool>()) {
    RCLCPP_INFO(get_logger(), "Self-transitioning to ACTIVE");
    change_state_request_->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE;
    change_state_future_ = change_state_client_->async_send_request(change_state_request_);
  }

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
BNO055Driver::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "%s is called.", __func__);

  diag_timer_.reset();
  imu_timer_.reset();
  tmp_pub_.reset();
  tmp_msg_.reset();
  mag_pub_.reset();
  mag_msg_.reset();
  imu_pub_.reset();
  imu_msg_.reset();

  // Switch to config mode
  port_->write_command_.address = BNO055Register::OPR_MODE;
  port_->write_command_.length = 1;
  port_->write_command_.data[0] = BNO055OperationMode::CONFIG;
  port_->write();

  port_->close();
  port_.reset();

  publish_diagnostics();
  diag_pub_.reset();
  diag_msg_.reset();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
BNO055Driver::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "%s is called.", __func__);

  // Switch operation mode
  port_->write_command_.address = BNO055Register::OPR_MODE;
  port_->write_command_.length = 1;
  auto operation_mode = use_magnetometer_ ? BNO055OperationMode::NDOF : BNO055OperationMode::IMU;
  port_->write_command_.data[0] = operation_mode;
  port_->write();

  imu_pub_->on_activate();
  if (use_magnetometer_) {
    mag_pub_->on_activate();
  }
  tmp_pub_->on_activate();
  diag_pub_->on_activate();

  publish_diagnostics();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
BNO055Driver::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "%s is called.", __func__);

  mag_pub_->on_deactivate();
  imu_pub_->on_deactivate();
  tmp_pub_->on_deactivate();

  // Switch to config mode
  port_->write_command_.address = BNO055Register::OPR_MODE;
  port_->write_command_.length = 1;
  port_->write_command_.data[0] = BNO055OperationMode::CONFIG;
  port_->write();

  // Now that we're back in config mode, pull the calibration values
  port_->read_command_.address = BNO055Register::ACC_OFFSET_X_LSB;
  port_->read_command_.length = 22;
  port_->read();
  std::vector<rclcpp::Parameter> calibration_parameters {
    rclcpp::Parameter(
      "calibration.accelerometer_offset",
      std::vector<int64_t> {
        bytes_to_short(&port_->read_response_.data[0]),
        bytes_to_short(&port_->read_response_.data[2]),
        bytes_to_short(&port_->read_response_.data[4]),
      }),
    rclcpp::Parameter(
      "calibration.magnetometer_offset",
      std::vector<int64_t> {
        bytes_to_short(&port_->read_response_.data[6]),
        bytes_to_short(&port_->read_response_.data[8]),
        bytes_to_short(&port_->read_response_.data[10]),
      }),
    rclcpp::Parameter(
      "calibration.gyroscope_offset",
      std::vector<int64_t> {
        bytes_to_short(&port_->read_response_.data[12]),
        bytes_to_short(&port_->read_response_.data[14]),
        bytes_to_short(&port_->read_response_.data[16]),
      }),
    rclcpp::Parameter(
      "calibration.accelerometer_radius",
      bytes_to_short(&port_->read_response_.data[18])),
    rclcpp::Parameter(
      "calibration.magnetometer_radius",
      bytes_to_short(&port_->read_response_.data[20])),
  };
  set_parameters(calibration_parameters);

  publish_diagnostics();
  diag_pub_->on_deactivate();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
BNO055Driver::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "%s is called.", __func__);

  diag_timer_.reset();
  imu_timer_.reset();
  tmp_pub_.reset();
  tmp_msg_.reset();
  mag_pub_.reset();
  mag_msg_.reset();
  imu_pub_.reset();
  imu_msg_.reset();

  if (port_ && port_->isOpen()) {
    port_->close();
  }

  port_.reset();

  if (diag_msg_) {
    publish_diagnostics();
    diag_pub_.reset();
    diag_msg_.reset();
  }

  change_state_client_.reset();
  change_state_request_.reset();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
BNO055Driver::on_error(const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(get_logger(), "%s is called from %s.", __func__, previous_state.label().c_str());

  diag_timer_.reset();
  imu_timer_.reset();
  tmp_pub_.reset();
  tmp_msg_.reset();
  mag_pub_.reset();
  mag_msg_.reset();
  imu_pub_.reset();
  imu_msg_.reset();

  if (port_ && port_->isOpen()) {
    port_->close();
  }

  port_.reset();

  if (diag_msg_) {
    publish_diagnostics();
    diag_pub_.reset();
    diag_msg_.reset();
  }

  if (get_parameter("self_manage").get_value<bool>()) {
    RCLCPP_INFO(get_logger(), "Self-transitioning to INACTIVE");
    change_state_request_->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE;
    change_state_request_->transition.label = "";
    change_state_future_ = change_state_client_->async_send_request(change_state_request_);
  }

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void BNO055Driver::publish() try
{
  RCLCPP_DEBUG(get_logger(), "Querying for current IMU data");

  port_->read_command_.address = BNO055Register::ACC_DATA_X_LSB;
  port_->read_command_.length = 46;
  port_->read();

  rclcpp::Time stamp = now();

  if (imu_pub_->is_activated()) {
    imu_msg_->header.stamp = stamp;
    imu_msg_->angular_velocity.x = bytes_to_short(&port_->read_response_.data[12]) / vel_scale_;
    imu_msg_->angular_velocity.y = bytes_to_short(&port_->read_response_.data[14]) / vel_scale_;
    imu_msg_->angular_velocity.z = bytes_to_short(&port_->read_response_.data[16]) / vel_scale_;

    imu_msg_->linear_acceleration.x = bytes_to_short(&port_->read_response_.data[0]) / lacc_scale_;
    imu_msg_->linear_acceleration.y = bytes_to_short(&port_->read_response_.data[2]) / lacc_scale_;
    imu_msg_->linear_acceleration.z = bytes_to_short(&port_->read_response_.data[4]) / lacc_scale_;

    imu_msg_->orientation.w = bytes_to_short(&port_->read_response_.data[24]) / scale::quat;
    imu_msg_->orientation.x = bytes_to_short(&port_->read_response_.data[26]) / scale::quat;
    imu_msg_->orientation.y = bytes_to_short(&port_->read_response_.data[28]) / scale::quat;
    imu_msg_->orientation.z = bytes_to_short(&port_->read_response_.data[30]) / scale::quat;

    imu_pub_->publish(*imu_msg_);
  }

  if (mag_pub_->is_activated()) {
    mag_msg_->header.stamp = stamp;
    mag_msg_->magnetic_field.x = bytes_to_short(&port_->read_response_.data[6]) / mag_scale_;
    mag_msg_->magnetic_field.y = bytes_to_short(&port_->read_response_.data[8]) / mag_scale_;
    mag_msg_->magnetic_field.z = bytes_to_short(&port_->read_response_.data[10]) / mag_scale_;

    mag_pub_->publish(*mag_msg_);
  }

  if (tmp_pub_->is_activated()) {
    tmp_msg_->header.stamp = stamp;
    tmp_msg_->temperature = port_->read_response_.data[44] / temp_scale_;

    tmp_pub_->publish(*tmp_msg_);
  }
} catch (std::exception & e) {
  RCLCPP_ERROR(get_logger(), "Failed to poll and publish data: %s", e.what());
  deactivate();
}

void BNO055Driver::publish_diagnostics()
try
{
  RCLCPP_DEBUG(get_logger(), "Querying for current IMU diagnostics");

  diag_msg_->header.stamp = now();

  if (!port_ || !port_->isOpen()) {
    diag_msg_->status[0].level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    diag_msg_->status[0].message = "Disconnected";
    diag_msg_->status[0].values.resize(0);
  } else {
    port_->read_command_.address = BNO055Register::CALIB_STAT;
    port_->read_command_.length = 6;
    port_->read();

    const uint8_t calib_sys = (port_->read_response_.data[0] >> 0) & 0x03;
    const uint8_t calib_gyr = (port_->read_response_.data[0] >> 2) & 0x03;
    const uint8_t calib_acc = (port_->read_response_.data[0] >> 4) & 0x03;
    const uint8_t calib_mag = (port_->read_response_.data[0] >> 6) & 0x03;
    const uint8_t self_test = port_->read_response_.data[1] & 0x0F;
    const uint8_t sys_status = port_->read_response_.data[4];
    const uint8_t sys_error = port_->read_response_.data[5];

    diag_msg_->status[0].values.resize(7);
    diag_msg_->status[0].values[0].key = "Calibration (SYS)";
    diag_msg_->status[0].values[0].value = std::to_string(calib_sys);
    diag_msg_->status[0].values[1].key = "Calibration (GYR)";
    diag_msg_->status[0].values[1].value = std::to_string(calib_gyr);
    diag_msg_->status[0].values[2].key = "Calibration (ACC)";
    diag_msg_->status[0].values[2].value = std::to_string(calib_acc);
    diag_msg_->status[0].values[3].key = "Calibration (MAG)";
    diag_msg_->status[0].values[3].value = std::to_string(calib_mag);
    diag_msg_->status[0].values[4].key = "Self-Test Status";
    diag_msg_->status[0].values[4].value = std::to_string(self_test);
    diag_msg_->status[0].values[5].key = "System Status";
    diag_msg_->status[0].values[5].value = std::to_string(sys_status);
    diag_msg_->status[0].values[6].key = "System Error";
    diag_msg_->status[0].values[6].value = std::to_string(sys_error);

    if (sys_error) {
      diag_msg_->status[0].level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      diag_msg_->status[0].message = "BNO055 System Error";
    } else if (self_test != 0x0F) {
      diag_msg_->status[0].level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      diag_msg_->status[0].message = "Self-Test Failure";
    } else if (calib_sys < 1 || calib_gyr < 1 || calib_acc < 1 || calib_mag < 1) {
      diag_msg_->status[0].level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      diag_msg_->status[0].message = "Poorly Calibrated";
    } else {
      diag_msg_->status[0].level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      diag_msg_->status[0].message = "Operating Normally";
    }
  }

  diag_pub_->publish(*diag_msg_);
} catch (std::exception & e) {
  RCLCPP_ERROR(get_logger(), "Failed to poll and publish diagnostics: %s", e.what());
}

}  // namespace bno055_driver
