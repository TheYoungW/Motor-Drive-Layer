cmake_minimum_required(VERSION 3.16)
project(motor_drive_layer_consumer LANGUAGES CXX)

find_package(MotorDriveLayer CONFIG REQUIRED)

add_executable(motor_drive_layer_consumer main.cpp)
target_link_libraries(
  motor_drive_layer_consumer PRIVATE motorbridge::articore_runtime_cpp
)
