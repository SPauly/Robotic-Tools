#include <iostream>
#include <memory>
#include <string>

#include "dist_of_laser.h"
#include "time_difference.h"
#include "rad_conv.h"
#include "subroutine_base.h"

void ShowHelp(char** argv);
void RunTimeDifference(const double& arg1, const double& arg2);

int main(int argc, char* argv[]) {
  std::shared_ptr<roboto::SubroutineBase> sub = nullptr;

  if (argc < 2) {
    ShowHelp(argv);
    return 1;
  }

  if (std::string(argv[1]) == "timedif") {
    if (argc != 4) {
      std::cout << "Usage: " << argv[0]
                << " timedif <Distance 1 in Meters> <Distance 2 in Meters>"
                << std::endl;
      return 1;
    }

    /// TODO: Test that they are correct values and convert them properly
    double arg1, arg2;

    try {
      arg1 = std::stod(argv[2]);
      arg2 = std::stod(argv[3]);
    } catch (const std::invalid_argument& e) {
      std::cerr << "Invalid argument: Please provide valid numeric values for "
                   "distances."
                << std::endl;
      return 1;
    } catch (const std::out_of_range& e) {
      std::cerr
          << "Out of range: The provided values are too large or too small."
          << std::endl;
      return 1;
    }

    RunTimeDifference(arg1, arg2);

    return 0;
  } else if (std::string(argv[1]) == "distoflaser") {
    if (argc < 3) {
      std::cout << "Usage: " << argv[0]
                << " distoflaser <arg1> <arg2> ... <argn>" << std::endl;
      return 1;
    }

    sub = std::make_shared<roboto::DistOfLaser>(argc, argv);

    if (sub == nullptr) {
      std::cerr << "Failed to create DistOfLaser subroutine." << std::endl;
      return 1;
    }

    sub->Run();
    return 0;

  } else if (std::string(argv[1]) == "radconv") {
    if (argc != 4) {
      std::cout << "Usage: " << argv[0] << " radconv <value> <num|pi|deg>"
                << std::endl;

      return 1;
    }

    sub = std::make_shared<roboto::RadConv>(argv);

    if (sub == nullptr) {
      std::cerr << "Failed to create RadConv subroutine." << std::endl;
      return 1;
    }
    sub->Run();
  }

  return 0;
}

void RunTimeDifference(const double& arg1, const double& arg2) {
  std::cout << "Time for Distance 1: " << roboto::DistanceToTime(arg1) * 1e9
            << " ns" << std::endl;
  std::cout << "Time for Distance 2: " << roboto::DistanceToTime(arg2) * 1e9
            << " ns" << std::endl;

  std::cout << "Difference in Time: "
            << roboto::DistanceToTime(std::abs(arg1 - arg2)) * 1e9 << " in ns"
            << std::endl;
}

void ShowHelp(char** argv) {
  std::cout << "Usage: " << argv[0] << " <name of subroutine> <args...>"
            << std::endl;
  std::cout
      << "Available Subroutines:\n\t > timedif <Distance 1 in Meters> "
         "<Distance 2 in Meters> - calculates difference in time of "
         "both laser distances\n \t > distoflaser <arg1> <arg2> ... <argn> - "
         "The distance between two lasers at distances arg1...n\n \t > "
         "radconv <value> <num|pi|deg> converts given value to lazernumber, "
         "angle and rad"
      << std::endl;
}