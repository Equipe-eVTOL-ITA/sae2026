cmake_minimum_required(VERSION 3.5)
project(mission_2)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fsm REQUIRED)
find_package(drone_lib REQUIRED)
find_package(stdstates REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(custom_msgs REQUIRED)

# Include directories
include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR})

# Mission 2 executable
add_executable(mission_2 src/mission_2.cpp)

ament_target_dependencies(mission_2
  rclcpp
  Eigen3
  fsm
  drone_lib
  stdstates
  px4_msgs
  custom_msgs
)

install(TARGETS mission_2
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
