cmake_minimum_required(VERSION 3.5)

project(shisen_gazebo_plugins)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(shisen_interfaces REQUIRED)

link_directories(${gazebo_dev_LIBRARY_DIRS})

install(DIRECTORY "include" DESTINATION ".")

add_library(shisen_camera_plugin SHARED
  "src/camera_plugin.cpp")

target_include_directories(shisen_camera_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(shisen_camera_plugin
  OpenCV gazebo_dev gazebo_ros rclcpp shisen_interfaces)

target_link_libraries(shisen_camera_plugin CameraPlugin)

install(TARGETS shisen_camera_plugin
  DESTINATION "share/${PROJECT_NAME}/plugins")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(
  OpenCV gazebo_dev gazebo_ros rclcpp shisen_interfaces)

ament_export_include_directories("include")
ament_export_libraries(${PROJECT_NAME})

ament_package()
