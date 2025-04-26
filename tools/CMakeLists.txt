cmake_minimum_required(VERSION 3.10)

# set the project name
project(roboto CXX)

# set the C++ standard to C++20 
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(ROOT_PATH ${CMAKE_SOURCE_DIR}/)

# add the executable
add_executable(roboto
  ${ROOT_PATH}/tools/roboto.cpp
  ${ROOT_PATH}/tools/dist_of_laser.cpp
  ${ROOT_PATH}/tools/laser_pos.cpp
)

target_include_directories(roboto PUBLIC
  ${CMAKE_SOURCE_DIR}/include/
  ${CMAKE_CURRENT_SOURCE_DIR}/
  )