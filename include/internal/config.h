#ifndef INTERNAL_CONFIG_H
#define INTERNAL_CONFIG_H

#include <cmath>

namespace roboto
{
  namespace internal
  {
    static const double ANGLE_OF_LASER_DEGREE = 0.5; // in degrees
    static const double ANGLE_OF_LASER_RADIAN = ANGLE_OF_LASER_DEGREE * M_PI / 180.0; // in radians
  
  } // namespace internal
  
} // namespace roboto


#endif // INTERNAL_CONFIG_H