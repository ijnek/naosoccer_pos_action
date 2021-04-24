cmake_minimum_required(VERSION 3.5)
project(naosoccer_pos_action)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Boost COMPONENTS filesystem REQUIRED)
find_package(nao_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(linear src/linear.cpp)

ament_target_dependencies(linear
  rclcpp
  ament_index_cpp
  Boost
  nao_interfaces
  std_msgs)

install(
  DIRECTORY pos/
  DESTINATION share/${PROJECT_NAME}/pos/
)

install(TARGETS
  linear
  DESTINATION lib/${PROJECT_NAME}/)

ament_package()
