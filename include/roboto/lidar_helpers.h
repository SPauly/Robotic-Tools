#ifndef ROBOTO_LIDAR_HELPERS_H
#define ROBOTO_LIDAR_HELPERS_H

#include <cmath>
#include <type_traits>

#include "roboto/internal/config.h"
#include "roboto/internal/conversions.h"
#include "roboto/internal/physics_constants.h"

namespace roboto {
// forward declaration

namespace internal {
template <typename Derived>
class AngleBase;
}

// -- Laser Distance --

/// @brief Returns the time it takes for a laser to travel a distance in meters.
/// @param distance distance in meters
/// @return Time in seconds
inline const double DistanceToTime(const double& distance) {
  return distance / internal::LIGHTSPEED_IN_AIR;
}

// -- Angle between lasers --

/// @brief Calculate the distance between two lasers - originating from the same
/// source - after m meters (in meters). It uses basic trigonometry to figur out
/// the distance
/// @param dist_of_lasers_m distance the lasers travel before we measure the
/// distance between them
/// @param laser_deg Degree offset between the lasers standard is set in
/// config.h
/// @return Distance between the two lasers in meters
const double DistBetweenLasersM(
    const double& dist_of_lasers_m,
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

// -- Conversion between laser number, degree and radian --

class RadianType : public internal::AngleBase<RadianType> {
 public:
  using internal::AngleBase<RadianType>::AngleBase;
  using internal::AngleBase<RadianType>::operator=;

  // Templated conversion function
  template <typename OtherDerived>
  static constexpr double convertFromOther(double value) noexcept {
    if constexpr (std::is_same_v<OtherDerived, DegreeType>) {
      return internal::DegToRad(value);
    } else if constexpr (std::is_same_v<OtherDerived, LaserNumType>) {
      return internal::LaserNumToRad(value);
    } else {
      static_assert(sizeof(OtherDerived) == 0,
                    "Unsupported conversion to RadianType.");
    }
  }
};

class DegreeType : public internal::AngleBase<DegreeType> {
 public:
  using internal::AngleBase<DegreeType>::AngleBase;
  using internal::AngleBase<DegreeType>::operator=;

  // Templated conversion function
  template <typename OtherDerived>
  static constexpr double convertFromOther(double value) noexcept {
    if constexpr (std::is_same_v<OtherDerived, RadianType>) {
      return internal::RadToDeg(value);
    } else if constexpr (std::is_same_v<OtherDerived, LaserNumType>) {
      return internal::LaserNumToDeg(value);
    } else {
      static_assert(sizeof(OtherDerived) == 0,
                    "Unsupported conversion to DegreeType.");
    }
  }
};

class LaserNumType : public internal::AngleBase<LaserNumType> {
 public:
  using internal::AngleBase<LaserNumType>::AngleBase;
  using internal::AngleBase<LaserNumType>::operator=;

  // Templated conversion function
  template <typename OtherDerived>
  static constexpr double convertFromOther(double value) noexcept {
    if constexpr (std::is_same_v<OtherDerived, RadianType>) {
      return internal::RadToLaserNum(value);
    } else if constexpr (std::is_same_v<OtherDerived, DegreeType>) {
      return internal::DegToLaserNum(value);
    } else {
      static_assert(sizeof(OtherDerived) == 0,
                    "Unsupported conversion to DegreeType.");
    }
  }
};

namespace internal {

template <typename Derived>
class AngleBase {
 protected:
  double value_;

 public:
  constexpr AngleBase() = default;
  constexpr explicit AngleBase(double value) noexcept : value_(value) {}

  // Assignment from double
  constexpr Derived& operator=(double value) noexcept {
    static_cast<Derived*>(this)->value_ = value;
    return *static_cast<Derived*>(this);
  }

  // Conversion to double
  constexpr explicit operator double() const noexcept { return value_; }

  /// @brief returns the raw value of the angle (using nodiscard to give
  /// compiler warnings when return not used)
  /// @return raw value of the angle
  [[nodiscard]] constexpr double value() const noexcept { return value_; }

  // This ensures explicit conversion from other derived types that must provide
  // a converFromOther function. It also ensures no unnecessary copies of the
  // same Type
  template <typename OtherDerived,
            typename = std::enable_if_t<!std::is_same_v<Derived, OtherDerived>>>
  constexpr explicit AngleBase(const AngleBase<OtherDerived>& other) noexcept
      : value_(Derived::convertFromOther(other.value())) {}

  // Math operators
  constexpr Derived operator+(const Derived& other) const noexcept {
    return Derived(value_ + other.value_);
  }

  constexpr Derived operator-(const Derived& other) const noexcept {
    return Derived(value_ - other.value_);
  }

  constexpr Derived& operator+=(const Derived& other) noexcept {
    value_ += other.value_;
    return *static_cast<Derived*>(this);
  }

  constexpr Derived& operator-=(const Derived& other) noexcept {
    value_ -= other.value_;
    return *static_cast<Derived*>(this);
  }

  // Comparison
  constexpr bool operator==(const Derived& other) const noexcept {
    return value_ == other.value_;
  }

  constexpr bool operator!=(const Derived& other) const noexcept {
    return value_ != other.value_;
  }

  constexpr bool operator<(const Derived& other) const noexcept {
    return value_ < other.value_;
  }

  constexpr bool operator<=(const Derived& other) const noexcept {
    return value_ <= other.value_;
  }

  constexpr bool operator>(const Derived& other) const noexcept {
    return value_ > other.value_;
  }

  constexpr bool operator>=(const Derived& other) const noexcept {
    return value_ >= other.value_;
  }
};

}  // namespace internal

}  // namespace roboto

#endif  // ROBOTO_LIDAR_HELPERS_H