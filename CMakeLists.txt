cmake_minimum_required(VERSION 3.8)
project(cod_behavior)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(cod_behavior src/sentry_behavior.cpp)
ament_target_dependencies(cod_behavior
        rclcpp
        nav2_msgs
        behaviortree_cpp_v3
        example_interfaces
        rclcpp_action
        geometry_msgs
)
include_directories(
        include
)
install(TARGETS cod_behavior
        DESTINATION lib/${PROJECT_NAME}
)

ament_package()