cmake_minimum_required(VERSION 3.10)

# set the project name
project(roboto CXX)

# set the C++ standard to C++20 
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# add the executable
add_executable(roboto
  src/roboto.cpp
  src/dist_of_laser.cpp
  src/laser_pos.cpp
)

target_include_directories(roboto PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include/
  )