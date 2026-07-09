cmake_minimum_required(VERSION 3.16)
project(damiao_cpp VERSION 0.1.0 LANGUAGES CXX)

include(CTest)

add_library(damiao_cpp INTERFACE)
add_library(damiao::cpp ALIAS damiao_cpp)

target_compile_features(damiao_cpp INTERFACE cxx_std_17)
target_include_directories(
  damiao_cpp
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

add_library(
  damiao_runtime STATIC
  src/runtime.cpp
  src/dm_serial_bus_posix.cpp
  src/socketcan_bus.cpp
  src/dm_device_bus.cpp
  src/dm_device_shim.cpp
)
add_library(damiao::runtime ALIAS damiao_runtime)
target_compile_features(damiao_runtime PUBLIC cxx_std_17)
target_link_libraries(damiao_runtime PUBLIC damiao::cpp ${CMAKE_DL_LIBS})
target_include_directories(damiao_runtime PRIVATE ../third_party/dm_device/v1.1.0)
set_target_properties(damiao_runtime PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(motor_abi SHARED src/motor_abi.cpp)
target_compile_features(motor_abi PRIVATE cxx_std_17)
target_include_directories(motor_abi PRIVATE include)
target_link_libraries(motor_abi PRIVATE damiao::runtime)

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

  add_executable(motor_abi_smoke_test tests/abi_smoke_test.cpp)
  target_include_directories(motor_abi_smoke_test PRIVATE include)
  target_link_libraries(motor_abi_smoke_test PRIVATE motor_abi)
  add_test(NAME motor_abi_smoke_test COMMAND motor_abi_smoke_test)

  add_executable(dm_serial_codec_test tests/dm_serial_codec_test.cpp)
  target_link_libraries(dm_serial_codec_test PRIVATE damiao::runtime)
  add_test(NAME dm_serial_codec_test COMMAND dm_serial_codec_test)

  add_executable(socketcan_codec_test tests/socketcan_codec_test.cpp)
  target_link_libraries(socketcan_codec_test PRIVATE damiao::runtime)
  add_test(NAME socketcan_codec_test COMMAND socketcan_codec_test)

  add_executable(dm_device_test tests/dm_device_test.cpp)
  target_link_libraries(dm_device_test PRIVATE damiao::runtime)
  add_test(NAME dm_device_test COMMAND dm_device_test)
endif()
