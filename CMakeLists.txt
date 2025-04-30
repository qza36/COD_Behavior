cmake_minimum_required(VERSION 3.8...3.31)
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
find_package(rm_interfaces REQUIRED)

add_executable(cod_behavior src/sentry_behavior.cpp)
add_executable(checkNavResult src/CheckNavResult.cpp)
add_executable(msgtest test/msgTest.cpp)
ament_target_dependencies(cod_behavior
        rclcpp
        nav2_msgs
        behaviortree_cpp_v3
        example_interfaces
        rclcpp_action
        rm_interfaces
        geometry_msgs

)
ament_target_dependencies(checkNavResult
        rclcpp
        nav2_msgs
        example_interfaces
        rclcpp_action
        geometry_msgs
)
ament_target_dependencies(msgtest
        rm_interfaces
        rclcpp
)
include_directories(
        include
)
install(TARGETS cod_behavior
        DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS checkNavResult
        DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS msgtest
        DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
        launch cod_bt actions
        DESTINATION share/${PROJECT_NAME}
)
ament_package()