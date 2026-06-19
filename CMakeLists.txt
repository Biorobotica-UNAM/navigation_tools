cmake_minimum_required(VERSION 3.5)
project(navigation_tools)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(known_locations_interfaces REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# find_package(robotino_msgs REQUIRED)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
)

add_library(navigation_tools SHARED
  src/Navigation.cpp
)

ament_target_dependencies(navigation_tools
  rclcpp
  std_msgs
  nlohmann_json
  known_locations_interfaces
  std_srvs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS navigation_tools
  DESTINATION lib
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(navigation_tools)
ament_export_dependencies(
  rclcpp
  std_msgs
  nlohmann_json
  known_locations_interfaces
  std_srvs
  geometry_msgs
)

ament_package()