#ifndef TIME_DIFFERENCE_H
#define TIME_DIFFERENCE_H

#include "internal/physics_constants.h"

namespace roboto
{
static const double DistanceToTime(const double &distance) {
  return static_cast<double>(distance / internal::LIGHTSPEED_IN_AIR);
}
} // namespace roboto


#endif //TIME_DIFFERENCE_H