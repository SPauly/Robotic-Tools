#include "laser_pos.h"

#include <iostream>
#include <iomanip>

#include "internal/conversions.h"

namespace roboto {
LaserPos::LaserPos(char** argv) : SubroutineBase() {
  try {
    value_ = std::stod(argv[2]);
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
  }

  if (std::string(argv[3]) == "num") {
    type_ = input_type::NUM;
  } else if (std::string(argv[3]) == "pi") {
    type_ = input_type::RAD;
  }
}

void LaserPos::Run() { RunImpl(); }

void LaserPos::RunImpl() {
  std::cout << std::fixed << std::setprecision(3);
  std::cout << "Lazer Number \t | Degree \t | Radian" << std::endl;
  std::cout << "----------------------------------------" << std::endl;

  switch (type_) {
    case input_type::NUM:
      laser_pos_ = FromNum();
      break;

    case input_type::RAD:
      laser_pos_ = FromRad();
      break;
    case input_type::DEG:
      laser_pos_ = FromDeg();
      break;
    default:
      break;
  }

  std::cout << laser_pos_.num << "\t\t | ";
  std::cout << laser_pos_.deg << "\t | ";
  std::cout << laser_pos_.rad << "\t" << std::endl;
  std::cout << "----------------------------------------" << std::endl;
}

LaserPosType LaserPos::FromNum() {
  if (value_ < 0 || value_ > internal::NUM_LAZERS) {
    std::cerr << "Error: The provided value is out of range." << std::endl;
    return LaserPosType{};
  }

  LaserPosType laser_pos;
  laser_pos.num = static_cast<int>(value_);

  // num -> deg: -135 + num * 0.5
  laser_pos.deg = internal::LaserNumToDeg(value_);
  // num -> rad: deg * (M_PI / (360.0 / 2))
  laser_pos.rad = internal::DegToRad(laser_pos.deg);

  return laser_pos;
}

LaserPosType LaserPos::FromRad() {
  LaserPosType laser_pos;
  // rad -> deg: rad * (360.0 / 2) / M_PI
  laser_pos.deg = internal::RadToDeg(value_);

  if (laser_pos.deg < internal::ZERO_LAZER_DEG ||
      laser_pos.deg > internal::MAX_LAZER_DEG) {
    std::cerr << "Error: The provided value is out of range." << std::endl;
    return LaserPosType{};
  }

  // rad -> num: (deg - internal::ZERO_LAZER_DEG) /
  // internal::ANGLE_OF_LASER_DEGREE
  laser_pos.num = internal::DegToLaserNum(laser_pos.deg);
  laser_pos.rad = value_;

  return laser_pos;
}

LaserPosType LaserPos::FromDeg() {
  if (value_ < internal::ZERO_LAZER_DEG || value_ > internal::MAX_LAZER_DEG) {
    std::cerr << "Error: The provided value is out of range." << std::endl;
    return LaserPosType{};
  }

  LaserPosType laser_pos;

  // deg -> rad: deg * (M_PI / (360.0 / 2))
  laser_pos.rad = internal::DegToRad(value_);

  // deg -> num: (deg - internal::ZERO_LAZER_DEG) /
  // internal::ANGLE_OF_LASER_DEGREE
  laser_pos.num = internal::DegToLaserNum(value_);
  laser_pos.deg = value_;
  return laser_pos;
}

}  // namespace roboto
