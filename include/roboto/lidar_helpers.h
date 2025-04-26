#ifndef ROBOTO_LIDAR_HELPERS_H
#define ROBOTO_LIDAR_HELPERS_H

#include "roboto/internal/config.h"
#include "roboto/internal/conversions.h"
#include "roboto/internal/physics_constants.h"

namespace roboto {

/// @brief Calculate the distance between two lasers - originating from the same
/// source - after m meters (in meters). It uses basic trigonometry to figur out
/// the distance
/// @param dist_of_lasers_m distance the lasers travel before we measure the
/// distance between them
/// @param laser_deg Degree offset between the lasers standard is set in
/// config.h
/// @return Distance between the two lasers in meters
const double DistBetweenLasersM(
    const double &dist_of_lasers_m,
    const double laser_deg = roboto::internal::ANGLE_OF_LASER_DEGREE) {
  // Use cosine law to calculate the distance between two lasers
  // dist = sqrt(a^2 + b^2 - 2*a*b*cos(angle)) where we have a = b

  double temp =
      (2.0 * (dist_of_lasers_m * dist_of_lasers_m)) -
      (2 * (dist_of_lasers_m * dist_of_lasers_m) * std::cos(laser_deg));

  return std::sqrt(temp);
}

/// @brief Calculate the distance two lasers travel before
/// @param dist_bet_lasers_m Distance between the two lasers in meters
/// @param laser_deg Degree offset between the lasers standard is set in
/// config.h
/// @return Distance the lasers travel before we measure the distance between
/// them as dist_bet_lasers_m in meters
const double DistToLaserSeparationM(
    const double dist_bet_lasers_m,
    const double laser_deg = roboto::internal::ANGLE_OF_LASER_DEGREE) {
  // dist = sqrt(2(x^2) - 2(x^2)cos(angle)) -> x = sqrt(in^2 / 2(1 -
  // cos(angle)))

  double dist =
      (dist_bet_lasers_m * dist_bet_lasers_m) / (2 * (1 - std::cos(laser_deg)));

  return std::sqrt(dist);
}

}  // namespace roboto

#endif  // ROBOTO_LIDAR_HELPERS_H