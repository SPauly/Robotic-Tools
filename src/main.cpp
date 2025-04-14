#include <iostream>
#include <memory>

#include "time_difference.h"
#include "subroutine_base.h"

int main(int argc, char* argv[]){
  std::shared_ptr<roboto::SubroutineBase> sub = nullptr;

  if(argc < 2){
    std::cout << "Usage: " << argv[0] << " <name of subroutine> <args...>"
              << std::endl;
    std::cout
        << "Available Subroutines:\n \t timedif <Distance 1 in Meters> "
           "<Distance 2 in Meters> - calculates difference in time of "
           "both laser distances\n \t distoflaser <arg1> <arg2> ... <argn> - "
           "The distance between two lasers at distances arg1...n\n \t radconv "
           "<value> <#|a|r> converts given value to lazernumber, angle and rad"
        << std::endl;
    return 1;
  }

  // Factory-Method for calling correct subroutine

  if(argv[1] == "timedif") {
    if(argc < 4){
      std::cout << "Usage: " << argv[0] << " timedif <Distance 1 in Meters> <Distance 2 in Meters>"
              << std::endl;
      return 1;
    }

    /// TODO: Test that they are correct values and convert them properly
    double max, min;

    std::cout << "Difference in Time: " << roboto::DistanceToTime(max - min)
              << "in ns" << std::endl;
    return 0;
  } else if(argv[1] == "distoflaser") {
    if(argc < 4) {
      std::cout << "Usage: " << argv[0] << " distoflaser <arg1> <arg2> ... <argn>"
              << std::endl;
      return 1;
    }

    /// TODO: pars args and create subroutine
  } else if(argv[1] == "radconv") {
    if(argc != 4){
      std::cout << "Usage: " << argv[0] << " radconv <value> <#|a|r>"
              << std::endl;

      return 1;
    }

    /// TODO: Parse arguments and create subroutine
  }

  return 0;
}