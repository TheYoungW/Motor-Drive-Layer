set(ARTICORE_PINOCCHIO_HEADER_ROOT "" CACHE PATH
  "Architecture-independent Pinocchio 3.8 include root for cross builds")
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
if(ARTICORE_PINOCCHIO_HEADER_ROOT)
  if(NOT EXISTS
      "${ARTICORE_PINOCCHIO_HEADER_ROOT}/pinocchio/algorithm/aba.hpp")
    message(FATAL_ERROR
      "ARTICORE_PINOCCHIO_HEADER_ROOT does not contain Pinocchio headers")
  endif()
  file(STRINGS
    "${ARTICORE_PINOCCHIO_HEADER_ROOT}/pinocchio/config.hpp"
    ARTICORE_PINOCCHIO_VERSION_LINE
    REGEX "^# define PINOCCHIO_VERSION ")
  if(NOT ARTICORE_PINOCCHIO_VERSION_LINE MATCHES "\"3\\.8\\.0\"")
    message(FATAL_ERROR
      "ARM64 product builds require Pinocchio headers version 3.8.0")
  endif()
  # Pinocchio's algorithms used by Runtime are templates. Cross builds only
  # need its architecture-independent headers; no host Pinocchio library is
  # ever linked into the ARM64 service.
  set(ARTICORE_PINOCCHIO_INCLUDE_DIRS
    "${ARTICORE_PINOCCHIO_HEADER_ROOT}"
    "${ARTICORE_PINOCCHIO_HEADER_ROOT}/pinocchio/deprecated")
  set(ARTICORE_PINOCCHIO_COMPILE_DEFINITIONS
    BOOST_FUSION_INVOKE_MAX_ARITY=12
    BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS
    BOOST_MPL_LIMIT_LIST_SIZE=30
    BOOST_MPL_LIMIT_VECTOR_SIZE=30)
else()
  find_package(pinocchio 3.8 REQUIRED)
  if(NOT TARGET pinocchio::pinocchio_headers)
    message(FATAL_ERROR "Pinocchio's native C++ headers target was not found")
  endif()
  get_target_property(ARTICORE_PINOCCHIO_INCLUDE_DIRS
    pinocchio::pinocchio_headers INTERFACE_INCLUDE_DIRECTORIES)
  get_target_property(ARTICORE_PINOCCHIO_COMPILE_DEFINITIONS
    pinocchio::pinocchio_headers INTERFACE_COMPILE_DEFINITIONS)
endif()

add_library(articore_runtime STATIC
  src/gripper_product_profiles.cpp
  src/product_cartesian.cpp
  src/product_cartesian_validation.cpp
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
  src/yunyi_runtime.cpp
  src/yunyi_runtime_core.cpp
  src/yunyi_runtime_facade.cpp
)
add_library(articore::runtime ALIAS articore_runtime)

target_compile_features(articore_runtime PUBLIC cxx_std_17)
target_include_directories(articore_runtime
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
  PRIVATE ${ARTICORE_PINOCCHIO_INCLUDE_DIRS})
target_compile_definitions(articore_runtime PRIVATE
  ${ARTICORE_PINOCCHIO_COMPILE_DEFINITIONS})
target_link_libraries(articore_runtime
  PUBLIC articore::motor
  PRIVATE Eigen3::Eigen Boost::boost)
set_target_properties(articore_runtime PROPERTIES
  POSITION_INDEPENDENT_CODE ON
  CXX_VISIBILITY_PRESET hidden
  VISIBILITY_INLINES_HIDDEN YES)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(articore_runtime PRIVATE -Wall -Wextra -Wpedantic)
endif()

if(BUILD_TESTING)
  add_executable(articore_runtime_test tests/runtime_test.cpp)
  target_link_libraries(articore_runtime_test PRIVATE articore::runtime)
  add_test(NAME runtime_behavior COMMAND articore_runtime_test)

  add_executable(articore_runtime_facade_test tests/facade_test.cpp)
  target_link_libraries(articore_runtime_facade_test PRIVATE articore::runtime)
  add_test(NAME runtime_facade COMMAND articore_runtime_facade_test)
endif()
