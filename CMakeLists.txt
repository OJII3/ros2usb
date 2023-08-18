# =====================================================
# cmakeを書く手間を減らすため、ament_cmake_autoを使う
# =====================================================

cmake_minimum_required(VERSION 3.8)
project(ros2usb)

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()
ament_auto_add_library(${PROJECT_NAME} SHARED src/ros2usb.cpp)
ament_auto_add_executable(${PROJECT_NAME}_bin src/main.cpp)

# # メッセージの生成 
# rosidl_generate_interfaces(${PROJECT_NAME}_msgs
#   "msg/USBPacket.msg"
#   DEPENDENCIES std_msgs)

# # 同じパッケージ内でメッセージを使うために必要 (このパッケージ内でしか使う予定がないメッセージなので)
# rosidl_target_interfaces(${PROJECT_NAME} ${PROJECT_NAME}_msgs "rosidl_typesupport_cpp")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  set(ament_cmake_cpplint_FOUND TRUE)
endif()

ament_auto_package()
