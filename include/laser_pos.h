#ifndef LASER_POS_H
#define LASER_POS_H

#include <string>

#include "subroutine_base.h"
#include "internal/config.h"

namespace roboto {

struct LaserPosType {
  int num;
  double deg;
  double rad;
};

class LaserPos : public SubroutineBase {
 public:
  LaserPos(char **argv);
  virtual ~LaserPos() override = default;

  virtual void Run() override;

 protected:
  enum class input_type { NUM, RAD, DEG };

  virtual void RunImpl();

  LaserPosType FromNum();
  LaserPosType FromRad();
  LaserPosType FromDeg();

 private:
  double value_;
  LaserPosType laser_pos_;
  input_type type_ = input_type::DEG;
};
}  // namespace roboto

#endif  // LASER_POS_H