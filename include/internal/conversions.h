#ifndef INTERNAL_CONVERSIONS_H
#define INTERNAL_CONVERSIONS_H

#include "internal/config.h"

namespace roboto {
namespace internal {

/// @brief Converts laser number to degree. Based on Laser configuration set in
/// config.h. Faster than LaserNumToRad.
/// @param num laser number ranging from 0 to NUM_LAZERS
/// @return degree value (does not check for out of range)
inline const double LaserNumToDeg(const double &num) {
  // num -> deg: -135 + num * 0.5
  return ZERO_LAZER_DEG + num * ANGLE_OF_LASER_DEGREE;
}

/// @brief Converts laser number to radian. Based on Laser configuration set in
/// config.h Slower than DegToRad.
/// @param num laser number ranging from 0 to NUM_LAZERS
/// @return radian value (does not check for out of range)
inline const double LaserNumToRad(const double &num) {
  // num -> rad: deg * (M_PI / (360.0 / 2))
  return LaserNumToDeg(num) * M_PI / (360.0 / 2);
}

/// @brief Converts Radian to Degree
/// @param num Radian value
/// @return Degree value (does not check for out of range)
inline const double RadToDeg(const double &rad) {
  // rad -> deg: rad * (360.0 / 2) / M_PI
  return rad * (360.0 / 2) / M_PI;
}

/// @brief Converts Radian to laser number. Based on Laser configuration set in
/// config.h. Slower than DegToLaserNum.
/// @param rad Radian value
/// @return laser number (does not check for out of range)
inline const int RadToLaserNum(const double &rad) {
  // rad -> num: (deg - internal::ZERO_LAZER_DEG) /
  // internal::ANGLE_OF_LASER_DEGREE
  return static_cast<int>((RadToDeg(rad) - ZERO_LAZER_DEG) /
                          ANGLE_OF_LASER_DEGREE);
}

/// @brief Converts Degree to Radian.
/// @param deg Degree value
/// @return Radian value (does not check for out of range)
inline const double DegToRad(const double &deg) {
  // deg -> rad: deg * (M_PI / (360.0 / 2))
  return deg * M_PI / (360.0 / 2);
}

/// @brief Converts Degree to laser number. Based on Laser configuration set in
/// config.h
/// @param deg Degree value
/// @return laser number (does not check for out of range)
inline const int DegToLaserNum(const double &deg) {
  // deg -> num: (deg - internal::ZERO_LAZER_DEG) /
  // internal::ANGLE_OF_LASER_DEGREE
  return static_cast<int>((deg - ZERO_LAZER_DEG) / ANGLE_OF_LASER_DEGREE);
}

}  // namespace internal

}  // namespace roboto

#endif  // INTERNAL_CONVERSIONS_H