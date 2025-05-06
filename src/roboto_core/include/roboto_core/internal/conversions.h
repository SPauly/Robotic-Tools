#ifndef INTERNAL_CONVERSIONS_H
#define INTERNAL_CONVERSIONS_H

#include "roboto_core/internal/config.h"

namespace roboto {
namespace internal {

// Precomputed constants
constexpr double DEG_TO_RAD = M_PI / (360.0 / 2.0);
constexpr double RAD_TO_DEG = (360.0 / 2.0) / M_PI;

// Fast conversions

/// @brief Converts Radian to Degree
/// @param num Radian value
/// @return Degree value (does not check for out of range)
inline constexpr double RadToDeg(const double &rad) {
  // rad -> deg: rad * (360.0 / 2) / M_PI
  return rad * RAD_TO_DEG;
}

/// @brief Converts Degree to Radian.
/// @param deg Degree value
/// @return Radian value (does not check for out of range)
inline constexpr double DegToRad(const double &deg) {
  // deg -> rad: deg * (M_PI / (360.0 / 2))
  return deg * DEG_TO_RAD;
}

/// @brief Converts laser number to degree. Based on Laser configuration set in
/// config.h.
/// @param num laser number ranging from 0 to NUM_LAZERS
/// @return degree value (does not check for out of range)
inline constexpr double LaserNumToDeg(const double &num) {
  // num -> deg: -135 + num * 0.5
  return MIN_LAZER_DEG + num * ANGLE_OF_LASER_DEGREE;
}

/// @brief Converts laser number to radian. Based on Laser configuration set in
/// config.h
/// @param num laser number ranging from 0 to NUM_LAZERS
/// @return radian value (does not check for out of range)
inline constexpr double LaserNumToRad(const double &num) {
  // num -> rad: look at LaserNumToDeg
  return LaserNumToDeg(num) * DEG_TO_RAD;
}

/// @brief Converts Radian to laser number. Based on Laser configuration set in
/// config.h. Slower than DegToLaserNum.
/// @param rad Radian value
/// @return laser number (does not check for out of range)
inline constexpr int RadToLaserNum(const double &rad) {
  // rad -> num: (deg - internal::MIN_LAZER_DEG) /
  // internal::ANGLE_OF_LASER_DEGREE
  return static_cast<int>(RadToDeg(rad) - MIN_LAZER_DEG) /
         ANGLE_OF_LASER_DEGREE;
}

/// @brief Converts Degree to laser number. Based on Laser configuration set in
/// config.h
/// @param deg Degree value
/// @return laser number (does not check for out of range)
inline const int DegToLaserNum(const double &deg) {
  // deg -> num: (deg - internal::MIN_LAZER_DEG) /
  // internal::ANGLE_OF_LASER_DEGREE
  return static_cast<int>((deg - MIN_LAZER_DEG) / ANGLE_OF_LASER_DEGREE);
}

}  // namespace internal

}  // namespace roboto

#endif  // INTERNAL_CONVERSIONS_H