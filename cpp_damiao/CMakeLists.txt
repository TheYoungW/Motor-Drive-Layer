cmake_minimum_required(VERSION 3.16)
project(damiao_cpp VERSION 0.1.0 LANGUAGES CXX)

include(CTest)

# 公共头文件接口：向使用者传递头文件路径和 C++17 要求。
add_library(damiao_cpp INTERFACE)
add_library(damiao::cpp ALIAS damiao_cpp)

target_compile_features(damiao_cpp INTERFACE cxx_std_17)
target_include_directories(
  damiao_cpp
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Yunyi 的内部 C++ Motor 核心。协议、Motor、Controller 和双通道分组仍
# 分层维护，但不再构建独立 C ABI 动态库。
set(
  DAMIAO_RUNTIME_SOURCES
  src/runtime.cpp
  src/socketcan_fd_bus.cpp
)

add_library(damiao_runtime STATIC ${DAMIAO_RUNTIME_SOURCES})
add_library(damiao::runtime ALIAS damiao_runtime)
target_compile_features(damiao_runtime PUBLIC cxx_std_17)
target_link_libraries(damiao_runtime PUBLIC damiao::cpp)
set_target_properties(damiao_runtime PROPERTIES POSITION_INDEPENDENT_CODE ON)

# 测试目标：仅在 BUILD_TESTING 开启时参与构建并注册到 CTest。
if(BUILD_TESTING)
  add_executable(damiao_protocol_test tests/protocol_test.cpp)
  target_link_libraries(damiao_protocol_test PRIVATE damiao::cpp)
  add_test(NAME damiao_protocol_test COMMAND damiao_protocol_test)

  add_executable(damiao_motor_test tests/motor_test.cpp)
  target_link_libraries(damiao_motor_test PRIVATE damiao::cpp)
  add_test(NAME damiao_motor_test COMMAND damiao_motor_test)

  add_executable(damiao_runtime_test tests/runtime_test.cpp)
  target_link_libraries(damiao_runtime_test PRIVATE damiao::runtime)
  add_test(NAME damiao_runtime_test COMMAND damiao_runtime_test)

  add_executable(socketcan_fd_test tests/socketcan_fd_test.cpp)
  target_link_libraries(socketcan_fd_test PRIVATE damiao::runtime)
  add_test(NAME socketcan_fd_test COMMAND socketcan_fd_test)
endif()
