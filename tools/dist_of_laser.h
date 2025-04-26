#ifndef DIST_OF_LASER_H
#define DIST_OF_LASER_H

#include <vector>

#include "subroutine_base.h"

namespace roboto {

class DistOfLaser : public SubroutineBase {
 public:
  DistOfLaser(int argc, char** argv);
  virtual ~DistOfLaser() override = default;

  virtual void Run() override;

 protected:
  const double CalcDistBetweenLasersCM(const double& dist_in_meters) const;

  const double CalcDistToLaserM(const double& dist_in_cm) const;

 private:
  std::vector<double> distances_;

  bool inv = false;
};

}  // namespace roboto

#endif  // DIST_OF_LASER_H