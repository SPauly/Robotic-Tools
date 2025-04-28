#include "laser_pos/laser_pos.h"

#include <iostream>
#include <iomanip>

namespace roboto {
LaserPos::LaserPos(char** argv) : SubroutineBase() {
  try {
    value_ = std::stod(argv[2]);
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
  }

  if (std::string(argv[3]) == "num") {
    type_ = input_type::NUM;
  } else if (std::string(argv[3]) == "rad") {
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

  LaserPosType laser_pos{LaserNumType(value_)};
  return laser_pos;
}

LaserPosType LaserPos::FromRad() {
  LaserPosType laser_pos{RadianType(value_)};

  if (laser_pos.deg < internal::MIN_LAZER_DEG ||
      laser_pos.deg > internal::MAX_LAZER_DEG) {
    std::cerr << "Error: The provided value is out of range." << std::endl;
    return LaserPosType{};
  }

  return laser_pos;
}

LaserPosType LaserPos::FromDeg() {
  if (value_ < internal::MIN_LAZER_DEG || value_ > internal::MAX_LAZER_DEG) {
    std::cerr << "Error: The provided value is out of range." << std::endl;
    return LaserPosType{};
  }

  LaserPosType laser_pos{DegreeType(value_)};
  return laser_pos;
}

}  // namespace roboto
