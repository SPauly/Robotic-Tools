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

class RadianType;
class DegreeType;
class LaserNumType;
class LaserPosType;

// -- Laser Distance --

/// @brief Returns the time it takes for a laser to travel a distance in meters.
/// @param distance distance in meters
/// @return Time in seconds
[[nodiscard]] inline constexpr double DistanceToTime(
    const double& distance) noexcept {
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
[[nodiscard]] inline constexpr double DistBetweenLasersM(
    const double dist_of_lasers_m,
    const double laser_deg = roboto::internal::ANGLE_OF_LASER_DEGREE) noexcept {
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
[[nodiscard]] inline const double DistToLaserSeparationM(
    const double dist_bet_lasers_m,
    const double laser_deg = roboto::internal::ANGLE_OF_LASER_DEGREE) noexcept {
  // dist = sqrt(2(x^2) - 2(x^2)cos(angle)) -> x = sqrt(in^2 / 2(1 -
  // cos(angle)))

  double dist =
      (dist_bet_lasers_m * dist_bet_lasers_m) / (2 * (1 - std::cos(laser_deg)));

  return std::sqrt(dist);
}

// -- Conversion between laser number, degree and radian --

namespace internal {

/// @brief Base class for angle types. It provides the basic functionality for
/// calculations with angles and conversions between them. Conversion
/// functionality must be provided in the derived classes via the
/// convertFromOther function.
/// @tparam Derived The derived type (RadianType, DegreeType, LaserNumType) ->
/// this uses Curiously Recurring Template Pattern (CRTP)
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

  // Function to be overloaded in derived classes for conversion
  template <typename OtherDerived>
  static constexpr double convertFromOther(double value) noexcept {
    // This function should be specialized in the derived classes
    static_assert(sizeof(Derived) == 0,
                  "convertFromOther must be specialized in derived classes.");
    return 0.0;
  }

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

/// @brief Class representing an angle in radians. It provides conversion from
/// DegreeType and LaserNumType. It also provides basic arithmetic operations.
/// @details The class is derived from the AngleBase class, which provides basic
/// arithmetic operations and comparison operators
class RadianType : public internal::AngleBase<RadianType> {
 public:
  using internal::AngleBase<RadianType>::AngleBase;
  using internal::AngleBase<RadianType>::operator=;

  // Templated conversion function

  /// @brief Converts from other angle types (DegreeType, LaserNumType) to
  /// RadianType.
  /// @tparam OtherDerived The type to convert from (DegreeType or
  /// LaserNumType).
  /// @param value Value to convert.
  /// @return Value in radians.
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

/// @brief Class representing an angle in degrees. It provides conversion from
/// RadianType and LaserNumType. It also provides basic arithmetic operations.
/// @details The class is derived from the AngleBase class, which provides basic
/// arithmetic operations and comparison operators.
class DegreeType : public internal::AngleBase<DegreeType> {
 public:
  using internal::AngleBase<DegreeType>::AngleBase;
  using internal::AngleBase<DegreeType>::operator=;

  // Templated conversion function

  /// @brief Converts from other angle types (RadianType, LaserNumType) to
  /// DegreeType.
  /// @tparam OtherDerived RadianType or LaserNumType.
  /// @param value value to convert.
  /// @return converted value in degrees.
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

/// @brief Class representing a laser number. It provides conversion from
/// RadianType and DegreeType. It also provides basic arithmetic operations.
/// @details The class is derived from the AngleBase class, which provides basic
/// arithmetic operations and comparison operators.
class LaserNumType : public internal::AngleBase<LaserNumType> {
 public:
  using internal::AngleBase<LaserNumType>::AngleBase;
  using internal::AngleBase<LaserNumType>::operator=;

  // Templated conversion function

  /// @brief Converts from other angle types (RadianType, DegreeType) to
  /// LaserNumType.
  /// @tparam OtherDerived RadianType or DegreeType.
  /// @param value value to convert.
  /// @return converted value in laser number.
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

/// @brief DTO representing the laser position. It contains the laser number,
/// degree and radian values. It provides safe creation of the object using
/// RadianType, DegreeType and LaserNumType. But will not check for out of range
/// values nor ensure safe editing of the values. Ones they are changed after
/// creation there is no guarantee that num, deg and rad are consistent.
struct LaserPosType {
  double num;  // laser number
  double deg;  // degree value
  double rad;  // radian value

  // Constructor
  constexpr LaserPosType() : num(0), deg(0), rad(0) {}

  template <typename T>
  constexpr explicit LaserPosType(T value)
    requires(std::is_same_v<T, RadianType> || std::is_same_v<T, DegreeType> ||
             std::is_same_v<T, LaserNumType>)
  {
    if constexpr (std::is_same_v<T, RadianType>) {
      num = internal::RadToLaserNum(value.value());
      deg = internal::RadToDeg(value.value());
      rad = value.value();
    } else if constexpr (std::is_same_v<T, DegreeType>) {
      num = internal::DegToLaserNum(value.value());
      deg = value.value();
      rad = internal::DegToRad(value.value());
    } else if constexpr (std::is_same_v<T, LaserNumType>) {
      num = value.value();
      deg = internal::LaserNumToDeg(value.value());
      rad = internal::LaserNumToRad(value.value());
    } else {
      static_assert(sizeof(T) == 0,
                    "Unsupported type for LaserPosType. Use "
                    "LaserPosType(RadianType) constructor instead.");
    }
  }
};

}  // namespace roboto

#endif  // ROBOTO_LIDAR_HELPERS_H