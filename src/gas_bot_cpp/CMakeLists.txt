cmake_minimum_required(VERSION 3.8)
project(gas_bot_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(gas_simulator src/gas_sim.cpp)
ament_target_dependencies(gas_simulator rclcpp std_msgs nav_msgs)

add_executable(bayes_filter src/bayes_filter.cpp)
ament_target_dependencies(bayes_filter rclcpp std_msgs nav_msgs geometry_msgs)

install(TARGETS
  gas_simulator
  bayes_filter
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()