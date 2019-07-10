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

#ifndef BNO055_DRIVER__BNO055_REG_HPP_
#define BNO055_DRIVER__BNO055_REG_HPP_

#include <stdint.h>

#include "bno055_driver/visibility_control.hpp"

namespace bno055_driver
{

enum BNO055RegisterCommand : uint8_t
{
  WRITE = 0,
  READ,
};

enum BNO055OperationMode : uint8_t
{
  CONFIG = 0,
  ACCONLY,
  MAGONLY,
  GYROONLY,
  ACCMAG,
  ACCGYRO,
  MAGGYRO,
  AMG,
  IMU,
  COMPASS,
  M4G,
  NDOF_FMC_OFF,
  NDOF,
};

enum BNO055PowerMode : uint8_t
{
  NORMAL = 0,
  LOW_POWER,
  SUSPEND,
};

enum BNO055MessageType : uint8_t
{
  REGISTER_COMMAND = 0xAA,
  READ_RESPONSE = 0xBB,
  RESPONSE_STATUS = 0xEE,
};

enum BNO055Register : uint8_t
{
  CHIP_ID = 0x00,
  ACC_ID,
  MAG_ID,
  GYR_ID,
  SW_REV_ID_LSB,
  SW_REV_ID_MSB,
  BL_REV_ID,
  PAGE_ID,
  ACC_DATA_X_LSB,

  MAG_DATA_X_LSB = 0x0E,

  GYR_DATA_X_LSB = 0x14,

  QUA_DATA_W_LSB = 0x20,

  LIA_DATA_X_LSB = 0x28,

  TEMP = 0x34,
  CALIB_STAT,
  ST_RESULT,
  INT_STA,
  SYS_CLK_STATUS,
  SYS_STATUS,
  SYS_ERR,
  UNIT_SEL,

  OPR_MODE = 0x3D,
  PWR_MODE,
  SYS_TRIGGER,
  TEMP_SOURCE,
  AXIS_MAP_CONFIG,
  AXIS_MAP_SIGN,

  ACC_OFFSET_X_LSB = 0x55,
  MAG_OFFSET_X_LSB = 0x5B,
  GYR_OFFSET_X_LSB = 0x61,
  ACC_RADIUS_LSB = 0x67,
  MAG_RADIUS_LSB = 0x69,
};

enum BNO055ResponseStatus : uint8_t
{
  UNKNOWN = 0,
  WRITE_SUCCESS,
  READ_FAIL,
  WRITE_FAIL,
  REGMAP_INVALID_ADDRESS,
  REGMAP_WRITE_DISABLED,
  WRONG_START_BYTE,
  BUS_OVER_RUN_ERROR,
  MAX_LENGTH_ERROR,
  MIN_LENGTH_ERROR,
  RECEIVE_CHARACTER_TIMEOUT,
};

}  // namespace bno055_driver

#endif  // BNO055_DRIVER__BNO055_REG_HPP_
