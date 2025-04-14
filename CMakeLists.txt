cmake_minimum_required(VERSION 3.10)

# set the project name
project(Robotic-Tools CXX)

# set the C++ standard to C++20 
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# add the executable
add_executable(Robotic-Tools
  src/roboto.cpp
  src/dist_of_laser.cpp
)

target_include_directories(Robotic-Tools PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include/
  )