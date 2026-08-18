cmake_minimum_required(VERSION 3.16)
project(articore_runtime LANGUAGES CXX)

find_package(pinocchio REQUIRED)
get_target_property(
  ARTICORE_BOOST_SERIALIZATION_LIBRARY
  Boost::serialization IMPORTED_LOCATION_RELEASE
)
if(NOT ARTICORE_BOOST_SERIALIZATION_LIBRARY)
  get_target_property(
    ARTICORE_BOOST_SERIALIZATION_LIBRARY
    Boost::serialization IMPORTED_LOCATION
  )
endif()
get_filename_component(
  ARTICORE_BOOST_LIBRARY_DIR "${ARTICORE_BOOST_SERIALIZATION_LIBRARY}" DIRECTORY
)
file(GLOB ARTICORE_BOOST_SERIALIZATION_SONAMES
  "${ARTICORE_BOOST_LIBRARY_DIR}/libboost_serialization.so.*"
)
list(LENGTH ARTICORE_BOOST_SERIALIZATION_SONAMES ARTICORE_BOOST_SONAME_COUNT)
if(UNIX AND NOT APPLE AND ARTICORE_BOOST_SONAME_COUNT EQUAL 0)
  message(FATAL_ERROR "Pinocchio's versioned Boost.Serialization runtime was not found")
endif()
if(ARTICORE_BOOST_SONAME_COUNT GREATER 0)
  list(GET ARTICORE_BOOST_SERIALIZATION_SONAMES 0 ARTICORE_BOOST_SERIALIZATION_SONAME)
endif()

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
  src/runtime_safety.cpp
  src/runtime_worker.cpp
)
add_library(articore::runtime_core ALIAS articore_runtime_core)
target_include_directories(articore_runtime_core PUBLIC include src)
target_compile_features(articore_runtime_core PUBLIC cxx_std_17)
target_link_libraries(articore_runtime_core PUBLIC pinocchio::pinocchio_default)
set_target_properties(
  articore_runtime_core PROPERTIES
  POSITION_INDEPENDENT_CODE ON
  CXX_VISIBILITY_PRESET hidden
  VISIBILITY_INLINES_HIDDEN YES
)

add_library(articore_runtime SHARED src/runtime_abi.cpp)
add_library(articore::runtime ALIAS articore_runtime)
target_link_libraries(
  articore_runtime PRIVATE articore::runtime_core pinocchio::pinocchio_default
)
target_include_directories(
  articore_runtime
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE src
)
target_compile_features(articore_runtime PUBLIC cxx_std_17)
target_compile_definitions(
  articore_runtime PRIVATE ARTICORE_RUNTIME_BUILDING_LIBRARY
)
if(UNIX AND NOT APPLE)
  target_link_options(articore_runtime PRIVATE "-Wl,--exclude-libs,ALL")
  set_target_properties(
    articore_runtime PROPERTIES
    BUILD_RPATH "\$ORIGIN:\$ORIGIN/dm_device:\$ORIGIN/robotics"
    INSTALL_RPATH "\$ORIGIN:\$ORIGIN/dm_device:\$ORIGIN/robotics"
    BUILD_WITH_INSTALL_RPATH YES
  )
  add_custom_command(
    TARGET articore_runtime POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory
            "$<TARGET_FILE_DIR:articore_runtime>/robotics"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "$<TARGET_FILE:pinocchio::pinocchio_default>"
            "$<TARGET_FILE_DIR:articore_runtime>/robotics/"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${ARTICORE_BOOST_SERIALIZATION_SONAME}"
            "$<TARGET_FILE_DIR:articore_runtime>/robotics/"
    VERBATIM
  )
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|amd64|AMD64)$")
    set(ARTICORE_PRIVATE_LIBSTDCXX
      "${CMAKE_SOURCE_DIR}/third_party/libstdcxx/linux/x86_64/libstdc++.so.6"
    )
    add_custom_command(
      TARGET articore_runtime POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_if_different
              "${ARTICORE_PRIVATE_LIBSTDCXX}"
              "$<TARGET_FILE_DIR:articore_runtime>/robotics/"
      VERBATIM
    )
  endif()
elseif(APPLE)
  set_target_properties(
    articore_runtime PROPERTIES
    BUILD_RPATH "@loader_path"
    INSTALL_RPATH "@loader_path"
  )
endif()
set_target_properties(
  articore_runtime PROPERTIES
  WINDOWS_EXPORT_ALL_SYMBOLS ON
  OUTPUT_NAME articore_runtime
  CXX_VISIBILITY_PRESET hidden
  VISIBILITY_INLINES_HIDDEN YES
)

add_library(articore_runtime_cpp INTERFACE)
add_library(articore::runtime_cpp ALIAS articore_runtime_cpp)
target_compile_features(articore_runtime_cpp INTERFACE cxx_std_17)
target_link_libraries(
  articore_runtime_cpp INTERFACE articore_runtime motorbridge::motor_abi
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
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
if(UNIX AND NOT APPLE)
  install(
    FILES
      "$<TARGET_FILE:pinocchio::pinocchio_default>"
      "${ARTICORE_BOOST_SERIALIZATION_SONAME}"
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/robotics"
  )
  if(ARTICORE_PRIVATE_LIBSTDCXX)
    install(
      FILES "${ARTICORE_PRIVATE_LIBSTDCXX}"
      DESTINATION "${CMAKE_INSTALL_LIBDIR}/robotics"
    )
  endif()
endif()

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
endif()
