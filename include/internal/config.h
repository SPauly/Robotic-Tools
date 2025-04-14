#ifndef INTERNAL_CONFIG_H
#define INTERNAL_CONFIG_H

#include <cmath>

namespace roboto {
namespace internal {

// Lidar setup
static const double ANGLE_OF_LASER_DEGREE = 0.5;  // in degrees
static const double ANGLE_OF_LASER_RADIAN =
    ANGLE_OF_LASER_DEGREE * M_PI / (360.0 / 2);  // in radians

static const double ZERO_LAZER_DEG = -135.0;  // in degrees
static const double MAX_LAZER_DEG = 135.0;    // in degrees
static const double LAZER_RANGE_DEG = 270.0;  // in degrees
static const double NUM_LAZERS = 540.0;       // number of lasers

}  // namespace internal

}  // namespace roboto

#endif  // INTERNAL_CONFIG_H