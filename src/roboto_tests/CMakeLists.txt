cmake_minimum_required(VERSION 3.8)
project(roboto_tests)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(roboto_core REQUIRED)
find_package(GTest REQUIRED)

set(ROBOTO_TESTS
  lidar_helpers_test
)

# Erzeuge für jeden Test ein eigenes Target
foreach(target ${ROBOTO_TESTS})
  add_executable(${target} src/${target}.cpp)
  target_include_directories(${target} PRIVATE 
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
    )
  target_link_libraries(${target} GTest::gtest_main)
  
  # Discover the tests
  ament_add_gtest(${target} src/${target}.cpp)  
  ament_target_dependencies(${target} roboto_core)

  # Install
  install(TARGETS ${target}
    EXPORT export_${PROJECT_NAME}
    DESTINATION lib/${PROJECT_NAME}
  )
endforeach()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
