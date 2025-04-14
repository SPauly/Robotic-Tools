#include "rad_conv.h"

#include <iostream>
#include <iomanip>

namespace roboto {
RadConv::RadConv(char** argv) : SubroutineBase() {
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

void RadConv::Run() {
  std::cout << std::fixed << std::setprecision(3);
  std::cout << "Lazer Number \t | Degree \t | Radian" << std::endl;
  std::cout << "----------------------------------------" << std::endl;

  switch (type_) {
    case input_type::NUM:
      FromNum();
      break;

    case input_type::RAD:
      FromRad();
      break;
    case input_type::DEG:
      FromDeg();
      break;
    default:
      break;
  }
}

void RadConv::FromNum() {
  // num -> deg: -135 + num * 0.5
  double deg =
      internal::ZERO_LAZER_DEG + value_ * internal::ANGLE_OF_LASER_DEGREE;
  // num -> rad: deg * (M_PI / (360.0 / 2))
  double rad = deg * M_PI / (360.0 / 2);

  std::cout << std::setprecision(3);
  std::cout << static_cast<int>(value_) << "\t\t | ";
  std::cout << deg << "\t | ";
  std::cout << rad << "\t" << std::endl;
  std::cout << "----------------------------------------" << std::endl;
}

void RadConv::FromRad() {
  // rad -> deg: rad * (360.0 / 2) / M_PI
  double deg = value_ * (360.0 / 2) / M_PI;

  if (deg < internal::ZERO_LAZER_DEG || deg > internal::MAX_LAZER_DEG) {
    std::cerr << "Error: The provided value is out of range." << std::endl;
    return;
  }

  // rad -> num: (deg - internal::ZERO_LAZER_DEG) /
  // internal::ANGLE_OF_LASER_DEGREE
  int num = static_cast<int>((deg - internal::ZERO_LAZER_DEG) /
                             internal::ANGLE_OF_LASER_DEGREE);
  std::cout << std::setprecision(3);
  std::cout << num << "\t\t | ";
  std::cout << deg << "\t | ";
  std::cout << value_ << "\t" << std::endl;
  std::cout << "----------------------------------------" << std::endl;
}

void RadConv::FromDeg() {
  if (value_ < internal::ZERO_LAZER_DEG || value_ > internal::MAX_LAZER_DEG) {
    std::cerr << "Error: The provided value is out of range." << std::endl;
    return;
  }

  // deg -> rad: deg * (M_PI / (360.0 / 2))
  double rad = value_ * M_PI / (360.0 / 2);

  // deg -> num: (deg - internal::ZERO_LAZER_DEG) /
  // internal::ANGLE_OF_LASER_DEGREE
  int num = static_cast<int>((value_ - internal::ZERO_LAZER_DEG) /
                             internal::ANGLE_OF_LASER_DEGREE);

  std::cout << std::setprecision(3);
  std::cout << num << "\t\t | ";
  std::cout << value_ << "\t | ";
  std::cout << rad << "\t" << std::endl;
  std::cout << "----------------------------------------" << std::endl;
}

}  // namespace roboto
