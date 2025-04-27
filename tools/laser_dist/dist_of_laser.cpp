#include "laser_dist/dist_of_laser.h"

#include <cmath>
#include <iostream>
#include <iomanip>
#include <string>

#include "roboto/internal/config.h"

namespace roboto {
DistOfLaser::DistOfLaser(int argc, char **argv, bool use_time_dif_hack)
    : SubroutineBase(), use_time_dif_hack_(use_time_dif_hack) {
  if (use_time_dif_hack) {
    RunTimeDifference(argv);
  } else {
    int i = 2;

    if (std::string(argv[2]) == "-i") {
      inv = true;
      ++i;
    }

    for (; i < argc; i++) {
      try {
        distances_.push_back(std::stod(argv[i]));
      } catch (const std::invalid_argument &e) {
        std::cerr
            << "Invalid argument: Please provide valid numeric values for "
               "distances."
            << std::endl;
        throw;
      } catch (const std::out_of_range &e) {
        std::cerr
            << "Out of range: The provided values are too large or too small."
            << std::endl;
        throw;
      }

      if (distances_.back() < 0) {
        std::cerr << "Invalid argument: Please provide positive numeric values "
                     "for distances."
                  << std::endl;
        throw std::invalid_argument("Negative distance provided.");
      }
    }
  }
}

void DistOfLaser::Run() {
  if (use_time_dif_hack_) {
    // This was already handled in the constructor
    // and the function RunTimeDifference has already printed the output.
    return;
  }
  if (!inv) {
    std::cout << "Distance of laser (m) \t | Distance Between lasers (m)"
              << std::endl;

    for (const auto &dist : distances_) {
      std::cout << std::fixed << std::setprecision(2);
      std::cout << dist << "\t\t\t | " << DistBetweenLasersM(dist) << std::endl;
    }
  } else {
    std::cout << "Distance Between lasers (m) \t | Distance from laser (m)"
              << std::endl;

    for (const auto &dist : distances_) {
      std::cout << std::fixed << std::setprecision(2);
      std::cout << dist << "\t\t\t\t | " << DistToLaserSeparationM << std::endl;
    }
  }
}

void DistOfLaser::RunTimeDifference(char **argv) {
  double arg1, arg2;

  try {
    arg1 = std::stod(argv[2]);
    arg2 = std::stod(argv[3]);
  } catch (const std::invalid_argument &e) {
    std::cerr << "Invalid argument: Please provide valid numeric values for "
                 "distances."
              << std::endl;
    return;
  } catch (const std::out_of_range &e) {
    std::cerr << "Out of range: The provided values are too large or too small."
              << std::endl;
    return;
  }

  std::cout << "Time for Distance 1: " << roboto::DistanceToTime(arg1) * 1e9
            << " ns" << std::endl;
  std::cout << "Time for Distance 2: " << roboto::DistanceToTime(arg2) * 1e9
            << " ns" << std::endl;

  std::cout << "Difference in Time: "
            << roboto::DistanceToTime(std::abs(arg1 - arg2)) * 1e9 << " in ns"
            << std::endl;
}

}  // namespace roboto
