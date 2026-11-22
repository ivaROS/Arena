cmake_minimum_required(VERSION 3.16)
project(gz_pedestrian_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(arena_people_msgs REQUIRED)
find_package(gz_transport_vendor REQUIRED)
find_package(gz-transport REQUIRED)
find_package(gz_msgs_vendor REQUIRED)
find_package(gz-msgs REQUIRED)

add_executable(gz_pedestrian_bridge src/gz_pedestrian_bridge.cpp)
ament_target_dependencies(gz_pedestrian_bridge rclcpp arena_people_msgs)
target_link_libraries(gz_pedestrian_bridge gz-transport::core gz-msgs::core)

install(TARGETS gz_pedestrian_bridge
  DESTINATION lib/${PROJECT_NAME})

ament_package()
