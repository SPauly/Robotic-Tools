#ifndef DIST_OF_LASER_H
#define DIST_OF_LASER_H

#include <vector>

#include "subroutine_base.h"
#include "roboto/lidar_helpers.h"

namespace roboto {

class DistOfLaser : public SubroutineBase {
 public:
  DistOfLaser(int argc, char** argv, bool use_time_dif_hack = false);
  virtual ~DistOfLaser() override = default;

  virtual void Run() override;

 protected:
  void RunTimeDifference(char** argv);

 private:
  std::vector<double> distances_;

  bool inv = false;
  bool use_time_dif_hack_ = false;
};

}  // namespace roboto

#endif  // DIST_OF_LASER_H