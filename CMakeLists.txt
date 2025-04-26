cmake_minimum_required(VERSION 3.10)

# set the project name
project(roboto CXX)

# set the C++ standard to C++20 
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# add the executable
add_subdirectory(tools)
