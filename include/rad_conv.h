#ifndef RAD_CONV_H
#define RAD_CONV_H

#include <string>

#include "subroutine_base.h"
#include "internal/config.h"

namespace roboto {
class RadConv : public SubroutineBase {
 public:
  RadConv(char **argv);
  ~RadConv() override = default;

  virtual void Run() override;

  enum class input_type { NUM, RAD, DEG };

 protected:
  void FromNum();
  void FromRad();
  void FromDeg();

 private:
  double value_;
  input_type type_ = input_type::DEG;
};
}  // namespace roboto

#endif  // RAD_CONV_H