cmake_minimum_required(VERSION 3.16)
project(articore_runtime LANGUAGES CXX)

find_package(pinocchio REQUIRED)
if(NOT TARGET pinocchio::pinocchio_headers)
  message(FATAL_ERROR "Pinocchio's native C++ headers target was not found")
endif()
get_target_property(
  ARTICORE_PINOCCHIO_INCLUDE_DIRS
  pinocchio::pinocchio_headers
  INTERFACE_INCLUDE_DIRECTORIES
)
get_target_property(
  ARTICORE_PINOCCHIO_COMPILE_DEFINITIONS
  pinocchio::pinocchio_headers
  INTERFACE_COMPILE_DEFINITIONS
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(
  articore_runtime_core STATIC
  src/gripper_product_profiles.cpp
  src/robot_model.cpp
  src/runtime.cpp
  src/runtime_commands.cpp
  src/runtime_enable.cpp
  src/runtime_gripper.cpp
  src/runtime_gravity.cpp
  src/runtime_joint_position.cpp
  src/runtime_maintenance.cpp
  src/runtime_safety.cpp
  src/runtime_trajectory.cpp
  src/runtime_worker.cpp
)
add_library(articore::runtime_core ALIAS articore_runtime_core)
target_include_directories(articore_runtime_core PUBLIC include)
target_compile_features(articore_runtime_core PUBLIC cxx_std_17)
target_include_directories(
  articore_runtime_core SYSTEM PRIVATE ${ARTICORE_PINOCCHIO_INCLUDE_DIRS}
)
target_link_libraries(articore_runtime_core PRIVATE Eigen3::Eigen Boost::boost)
target_compile_definitions(
  articore_runtime_core
  PRIVATE
    ${ARTICORE_PINOCCHIO_COMPILE_DEFINITIONS}
)
set_target_properties(
  articore_runtime_core PROPERTIES
  POSITION_INDEPENDENT_CODE ON
  CXX_VISIBILITY_PRESET hidden
  VISIBILITY_INLINES_HIDDEN YES
)

add_library(
  articore_runtime SHARED
  src/product_cartesian.cpp
  src/runtime_abi.cpp
  src/yunyi_runtime.cpp
)
add_library(articore::runtime ALIAS articore_runtime)
target_link_libraries(
  articore_runtime PRIVATE articore::runtime_core damiao::runtime
)
target_include_directories(articore_runtime PRIVATE ../cpp_damiao/include)
target_include_directories(
  articore_runtime
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_features(articore_runtime PUBLIC cxx_std_17)
target_link_options(
  articore_runtime
  PRIVATE
    "-Wl,--exclude-libs,ALL"
    "-Wl,--as-needed"
    "-Wl,-z,defs"
)
set_target_properties(
  articore_runtime PROPERTIES
  BUILD_RPATH "\$ORIGIN"
  INSTALL_RPATH "\$ORIGIN"
  OUTPUT_NAME articore_runtime
  CXX_VISIBILITY_PRESET hidden
  VISIBILITY_INLINES_HIDDEN YES
)
add_library(articore_runtime_cpp INTERFACE)
add_library(articore::runtime_cpp ALIAS articore_runtime_cpp)
target_compile_features(articore_runtime_cpp INTERFACE cxx_std_17)
target_link_libraries(
  articore_runtime_cpp INTERFACE articore_runtime
)
target_include_directories(
  articore_runtime_cpp
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

if(MSVC)
  target_compile_options(articore_runtime_core PRIVATE /W4)
else()
  target_compile_options(articore_runtime_core PRIVATE -Wall -Wextra -Wpedantic)
endif()

include(GNUInstallDirs)
install(
  TARGETS articore_runtime articore_runtime_cpp
  EXPORT MotorDriveLayerTargets
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
install(
  DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  PATTERN "detail" EXCLUDE
)
include(CTest)
if(BUILD_TESTING)
  add_executable(articore_runtime_test tests/runtime_test.cpp)
  target_link_libraries(articore_runtime_test PRIVATE articore_runtime_core)
  add_test(NAME articore_runtime_test COMMAND articore_runtime_test)

  add_executable(articore_runtime_abi_smoke_test tests/abi_smoke_test.cpp)
  target_link_libraries(articore_runtime_abi_smoke_test PRIVATE articore_runtime)
  add_test(
    NAME articore_runtime_abi_smoke_test
    COMMAND articore_runtime_abi_smoke_test
  )

  add_executable(articore_runtime_cpp_binding_test tests/cpp_binding_test.cpp)
  target_link_libraries(
    articore_runtime_cpp_binding_test PRIVATE articore::runtime_cpp
  )
  add_test(
    NAME articore_runtime_cpp_binding_test
    COMMAND articore_runtime_cpp_binding_test
  )

  add_executable(articore_robot_model_abi_test tests/robot_model_abi_test.cpp)
  target_link_libraries(articore_robot_model_abi_test PRIVATE articore_runtime)
  add_test(NAME articore_robot_model_abi_test COMMAND articore_robot_model_abi_test)

  # Built explicitly for guarded real-hardware acceptance; never registered
  # with CTest because it enables and moves both arms.
  add_executable(
    articore_runtime_hardware_trajectory_smoke EXCLUDE_FROM_ALL
    tests/hardware_trajectory_smoke.cpp
  )
  target_link_libraries(
    articore_runtime_hardware_trajectory_smoke PRIVATE articore_runtime
  )
  add_executable(
    articore_runtime_hardware_ptp_reference_diagnostic EXCLUDE_FROM_ALL
    tests/hardware_ptp_reference_diagnostic.cpp
  )
  target_link_libraries(
    articore_runtime_hardware_ptp_reference_diagnostic
    PRIVATE articore_runtime
  )
  add_executable(
    articore_runtime_hardware_motor_register_diagnostic EXCLUDE_FROM_ALL
    tests/hardware_motor_register_diagnostic.cpp
    src/yunyi_runtime.cpp
  )
  target_link_libraries(
    articore_runtime_hardware_motor_register_diagnostic
    PRIVATE articore_runtime_core damiao::runtime
  )
  target_include_directories(
    articore_runtime_hardware_motor_register_diagnostic
    PRIVATE ../cpp_damiao/include
  )
  add_executable(
    articore_runtime_hardware_500hz_smoke EXCLUDE_FROM_ALL
    tests/hardware_500hz_smoke.cpp
  )
  target_link_libraries(
    articore_runtime_hardware_500hz_smoke PRIVATE articore_runtime
  )
endif()
