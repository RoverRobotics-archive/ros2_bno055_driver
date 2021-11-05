#ifndef BNO055_DRIVER__BNO055_SCALES_HPP_
#define BNO055_DRIVER__BNO055_SCALES_HPP_

/**
 * All the constant scale values are from the BNO055 Data sheet.
 * 
 * All values are const double, because the ROS messages expect
 * double values as well.
 */
namespace bno055_driver
{
    namespace scale {
    /**
     * > Table 3-22: Gyroscope unit settings 
     * > 1 Dps = 16 LSB Rps 1 Rps = 900 LSB
     */
    constexpr double vel_to_dps = 16;
    constexpr double vel_to_rps = 900;

    /**
     * > Table 3-17: Accelerometer Unit settings
     * > 1 m/s2 = 100 LSB and 1 mg = 1 LSB
     *
     * > Table 3-35: Gravity Vector data representation 
     * > 1 m/s2 = 100 LSB and 1 mg = 1 LSB
     */
    constexpr double acc_to_mps2 = 100;
    constexpr double acc_to_mg = 1;
    constexpr double acc_to_g = 1000;

    /**
     * > Table 3-19: Magnetometer Unit settings 
     * > 1 μT = 16 LSB
     */
    constexpr double mag_to_ut = 16;
    constexpr double mag_to_t = 16000000;

    /**
     * > Table 3-29: Euler angle data representation 
     * > 1 degree = 16 LSB and 1 radian = 900 LSB 
     */
    constexpr double euler_to_deg = 16;
    constexpr double euler_to_rad = 900;

    /**
     * Table 3-31: Quaternion data representation 
     * 1 Quaternion (unit less) = 2^14 LSB 
     */
    constexpr double quat = 16384;

    /**
     * Table 3-37: Temperature data representation 
     * 1°C = 1 LSB and 2°F = 1 LSB
     */
    constexpr double temp_to_c = 1;
    constexpr double temp_to_f = 2;

    } // namespace scale

} // namespace bno055_driver

#endif  // BNO055_DRIVER__BNO055_SCALES_HPP_