#ifndef SUBROUTINE_BASE_H
#define SUBROUTINE_BASE_H

namespace roboto
{
class SubroutineBase {
  public:
   SubroutineBase() = default;
   virtual ~SubroutineBase() = default;

   virtual void Run() = 0;
};
}  // namespace roboto

#endif //SUBROUTINE_BASE_H