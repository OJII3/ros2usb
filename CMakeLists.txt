# =====================================================
# cmakeを書く手間を減らすため、ament_cmake_autoを使う
# =====================================================

cmake_minimum_required(VERSION 3.8)
project(ros2usb)

# add extension for generating messages and services

# find dependencies
find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()
ament_auto_add_library(${PROJECT_NAME}_lib SHARED src/ros2usb.cpp)
ament_auto_add_executable(${PROJECT_NAME}_bin src/ros2usb.cpp)

# rosidl_generate_interfaces(${PROJECT_NAME}_interfaces
#   "msg/USBPacket.msg"
#   DEPENDENCIES std_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  set(ament_cmake_cpplint_FOUND TRUE)
endif()

ament_auto_package()
