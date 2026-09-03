add_library(articore_motor STATIC
  src/runtime.cpp
  src/socketcan_fd_bus.cpp
)
add_library(articore::motor ALIAS articore_motor)
add_library(damiao::runtime ALIAS articore_motor)

target_compile_features(articore_motor PUBLIC cxx_std_17)
target_include_directories(articore_motor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>)
set_target_properties(articore_motor PROPERTIES
  POSITION_INDEPENDENT_CODE ON
  CXX_VISIBILITY_PRESET hidden
  VISIBILITY_INLINES_HIDDEN YES)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(articore_motor PRIVATE -Wall -Wextra -Wpedantic)
endif()

if(BUILD_TESTING)
  add_executable(damiao_protocol_test tests/protocol_test.cpp)
  target_link_libraries(damiao_protocol_test PRIVATE articore::motor)
  add_test(NAME motor_protocol COMMAND damiao_protocol_test)

  add_executable(damiao_motor_test tests/motor_test.cpp)
  target_link_libraries(damiao_motor_test PRIVATE articore::motor)
  add_test(NAME motor_behavior COMMAND damiao_motor_test)

  add_executable(damiao_runtime_test tests/runtime_test.cpp)
  target_link_libraries(damiao_runtime_test PRIVATE articore::motor)
  add_test(NAME motor_runtime COMMAND damiao_runtime_test)

  add_executable(socketcan_fd_test tests/socketcan_fd_test.cpp)
  target_link_libraries(socketcan_fd_test PRIVATE articore::motor)
  add_test(NAME motor_socketcan_fd COMMAND socketcan_fd_test)

  # Manual hardware diagnostic: it only requests feedback and never enables or
  # moves a motor, so keep it buildable without registering it as an automated test.
  add_executable(damiao_hardware_feedback_stress tests/hardware_feedback_stress.cpp)
  target_link_libraries(damiao_hardware_feedback_stress PRIVATE articore::motor)
endif()
