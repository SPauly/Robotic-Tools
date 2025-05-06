#include <gtest/gtest.h>
#include "roboto_core/lidar_helpers.h"

namespace roboto {
namespace roboto_test {

// Helper constants for tests
constexpr double kEpsilon = 1e-9;
constexpr double kLightspeed = internal::LIGHTSPEED_IN_AIR;
constexpr double kLaserAngleDeg = internal::ANGLE_OF_LASER_DEGREE;

// --- DistanceToTime ---
TEST(LidarHelpersTest, DistanceToTime) {
  double distance = 300.0;  // meters
  double expected_time = distance / kLightspeed;
  EXPECT_NEAR(DistanceToTime(distance), expected_time, kEpsilon);
}

// --- DistBetweenLasersM ---
TEST(LidarHelpersTest, DistBetweenLasersM) {
  double dist = 10.0;
  double angle_deg = 10.0;
  double angle_rad = internal::DegToRad(angle_deg);
  double expected =
      std::sqrt(2 * dist * dist - 2 * dist * dist * std::cos(angle_rad));
  EXPECT_NEAR(DistBetweenLasersM(dist, angle_rad), expected, kEpsilon);
}

// --- DistToLaserSeparationM ---
TEST(LidarHelpersTest, DistToLaserSeparationM) {
  double separation = 2.0;
  double angle_deg = 10.0;
  double angle_rad = internal::DegToRad(angle_deg);
  double expected =
      std::sqrt((separation * separation) / (2 * (1 - std::cos(angle_rad))));
  EXPECT_NEAR(DistToLaserSeparationM(separation, angle_rad), expected,
              kEpsilon);
}

// --- Angle Type Conversions ---
TEST(LidarHelpersTest, RadianToDegreeAndBack) {
  RadianType rad(3.14159265358979323846 / 2);  // pi/2
  DegreeType deg(rad);
  EXPECT_NEAR(deg.value(), 90.0, kEpsilon);

  RadianType rad2(deg);
  EXPECT_NEAR(rad2.value(), rad.value(), kEpsilon);
}

TEST(LidarHelpersTest, DegreeToLaserNumAndBack) {
  DegreeType deg(45.0);
  LaserNumType num(deg);
  DegreeType deg2(num);
  EXPECT_NEAR(deg2.value(), deg.value(), kEpsilon);
}

TEST(LidarHelpersTest, LaserNumToRadianAndBack) {
  LaserNumType num(540.0);
  DegreeType deg(num);
  EXPECT_NEAR(deg.value(), 135.0, kEpsilon);

  RadianType rad(num);
  EXPECT_NEAR(rad.value(), internal::DegToRad(135.0), kEpsilon);

  LaserNumType num2(rad);
  EXPECT_NEAR(num2.value(), num.value(), kEpsilon);
}

// --- AngleBase Arithmetic and Comparison ---
TEST(LidarHelpersTest, AngleBaseArithmetic) {
  DegreeType a(30.0), b(15.0);
  DegreeType c = a + b;
  EXPECT_NEAR(c.value(), 45.0, kEpsilon);

  c -= b;
  EXPECT_NEAR(c.value(), 30.0, kEpsilon);

  EXPECT_TRUE(a > b);
  EXPECT_TRUE(b < a);
  EXPECT_TRUE(a == DegreeType(30.0));
  EXPECT_TRUE(a != b);
}

// --- LaserPosType Construction ---
TEST(LidarHelpersTest, LaserPosTypeFromRadian) {
  RadianType rad(internal::DegToRad(60.0));
  LaserPosType pos(rad);
  EXPECT_NEAR(pos.rad, rad.value(), kEpsilon);
  EXPECT_NEAR(pos.deg, 60.0, kEpsilon);
  EXPECT_NEAR(pos.num, internal::DegToLaserNum(60.0), kEpsilon);
}

TEST(LidarHelpersTest, LaserPosTypeFromDegree) {
  DegreeType deg(120.0);
  LaserPosType pos(deg);
  EXPECT_NEAR(pos.deg, 120.0, kEpsilon);
  EXPECT_NEAR(pos.rad, internal::DegToRad(120.0), kEpsilon);
  EXPECT_NEAR(pos.num, internal::DegToLaserNum(120.0), kEpsilon);
}

TEST(LidarHelpersTest, LaserPosTypeFromLaserNum) {
  LaserNumType num(7.0);
  LaserPosType pos(num);
  EXPECT_NEAR(pos.num, 7.0, kEpsilon);
  EXPECT_NEAR(pos.deg, internal::LaserNumToDeg(7.0), kEpsilon);
  EXPECT_NEAR(pos.rad, internal::LaserNumToRad(7.0), kEpsilon);
}

}  // namespace roboto_test
}  // namespace roboto