cmake_minimum_required(VERSION 3.5)
project(dobot_hardware)

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
find_package(rclcpp REQUIRED)
find_package(controller_manager REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(dobot_bringup REQUIRED)


add_library(dobot_hardware
        SHARED
        src/dobot_hardware_interface.cpp
)


target_include_directories(
        dobot_hardware
        PUBLIC
        include
)


ament_target_dependencies(
        dobot_hardware
        dobot_bringup
        hardware_interface
        controller_manager
        controller_manager_msgs
        pluginlib
        rclcpp
)

pluginlib_export_plugin_description_file(hardware_interface dobot_hardware.xml)


install(
        TARGETS dobot_hardware
        DESTINATION lib
)

install(
        DIRECTORY include/
        DESTINATION include
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(hardware_interface REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()



ament_export_include_directories(
        include
)

ament_export_libraries(
        dobot_hardware
)
ament_export_dependencies(
        hardware_interface
        controller_manager_msgs
        pluginlib
        rclcpp
)

ament_package()
