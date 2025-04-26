#include <iostream>
#include <memory>
#include <string>

#include "laser_dist/dist_of_laser.h"
#include "laser_pos/laser_pos.h"
#include "subroutine_base.h"
#include "roboto/lidar_helpers.h"

#include "roboto/lidar_helpers.h"

void ShowHelp(char** argv);

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

    sub = std::make_shared<roboto::DistOfLaser>(argc, argv, true);
    if (sub == nullptr) {
      std::cerr << "Failed to create DistOfLaser subroutine." << std::endl;
      return 1;
    }
    sub->Run();

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

  } else if (std::string(argv[1]) == "laserpos") {
    if (argc != 4) {
      std::cout << "Usage: " << argv[0] << " laserpos <value> <num|pi|deg>"
                << std::endl;

      return 1;
    }

    sub = std::make_shared<roboto::LaserPos>(argv);

    if (sub == nullptr) {
      std::cerr << "Failed to create LaserPos subroutine." << std::endl;
      return 1;
    }
    sub->Run();
  } else {
    std::cerr << "Unknown subroutine: " << argv[1] << std::endl;
    ShowHelp(argv);
    return 1;
  }

  return 0;
}

void ShowHelp(char** argv) {
  std::cout << "Usage: " << argv[0] << " <name of subroutine> <args...>"
            << std::endl;
  std::cout
      << "Available Subroutines:\n\t > timedif <Distance 1 in Meters> "
         "<Distance 2 in Meters> - calculates difference in time of "
         "both laser distances\n \t > distoflaser <arg1> <arg2> ... <argn> - "
         "The distance between two lasers at distances arg1...n\n \t > "
         "laserpos <value> <num|rad|deg> converts given value to lazernumber, "
         "angle and rad"
      << std::endl;
}