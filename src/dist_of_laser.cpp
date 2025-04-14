#include "dist_of_laser.h"

#include <cmath>
#include <iostream>
#include <string>

#include "internal/config.h"

namespace roboto
{
  DistOfLaser::DistOfLaser(int argc, char **argv) : SubroutineBase()
  {
    int i = 2;

    if(std::string(argv[1]) == "-i")
    {
      inv = true;
      ++i;
    }

    for(; i < argc; ++i)
    {
      try
      {
        distances_.push_back(std::stod(argv[i]));
      }
      catch (const std::invalid_argument &e)
      {
        std::cerr << "Invalid argument: Please provide valid numeric values for "
                     "distances."
                  << std::endl;
        throw;
      }
      catch (const std::out_of_range &e)
      {
        std::cerr
            << "Out of range: The provided values are too large or too small."
            << std::endl;
        throw;
      }

      if (distances_.back() < 0)
      {
        std::cerr << "Invalid argument: Please provide positive numeric values "
                     "for distances."
                  << std::endl;
        throw std::invalid_argument("Negative distance provided.");
      }
    }
  }

  void DistOfLaser::Run()
  {

    if (!inv){
    std::cout << "Distance of laser (m) \t | Distance Between lasers (cm)" << std::endl;

    for (const auto &dist : distances_)
    {
      std::cout << dist << "\t\t\t | " << CalcLength(dist) << std::endl;
    }
  } 
  }

  const double DistOfLaser::CalcLength(const double & dist) const
  {
    // Use cosine law to calculate the distance between two lasers

    // dist = sqrt(a^2 + b^2 - 2*a*b*cos(angle))

    double temp =
        (2.0 * (dist * dist)) - (2 * (dist * dist) * std::cos(internal::ANGLE_OF_LASER_RADIAN));

    return std::sqrt(temp);
  }
} // namespace roboto
