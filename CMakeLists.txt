cmake_minimum_required(VERSION 3.10)

# set the project name
project(roboto CXX)

# set the C++ standard to C++20 
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

#---------------------------------#
# Setup GoogleTest
#---------------------------------#

# Fetch google test
include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
# Workaround for Windows:
# Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

# GoogleTest soll nur lokal, in diesem Projekt verwendet werden.
option(INSTALL_GTEST "Enable installation of googletest." OFF)

# Include CMake configuration of GoogleTest
FetchContent_MakeAvailable(googletest)
include(GoogleTest)

# Enable GoogleTest
enable_testing()


#---------------------------------#
# Build Tools
#---------------------------------#

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/tools)

#---------------------------------#
# Build Tests
#---------------------------------#

set(ROBOTO_TESTS
  lidar_helpers_test
)

# Erzeuge für jeden Test ein eigenes Target
foreach(target ${ROBOTO_TESTS})
  add_executable(${target} ${CMAKE_CURRENT_SOURCE_DIR}/test/${target}.cpp)
  target_include_directories(${target} PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}/tools
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
  target_link_libraries(${target} GTest::gtest_main)
  gtest_discover_tests(${target})
endforeach()
