cmake_minimum_required(VERSION 3.5)
project(waypoint_logger)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)
# cmake stuff goes here

add_executable(waypoint_vis src/waypoint_visualizer.cpp)

ament_target_dependencies(waypoint_vis rclcpp visualization_msgs geometry_msgs)

install(TARGETS
  waypoint_vis
  DESTINATION lib/${PROJECT_NAME})

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS 
  scripts/waypoint_logger.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
