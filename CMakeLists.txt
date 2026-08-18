cmake_minimum_required(VERSION 3.16)
project(motorbridge_native VERSION 0.10.12 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(CTest)

add_subdirectory(cpp_damiao)
add_subdirectory(articore_runtime)

include(CMakePackageConfigHelpers)
include(GNUInstallDirs)
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/MotorDriveLayerConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)
configure_package_config_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/MotorDriveLayerConfig.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/MotorDriveLayerConfig.cmake"
  INSTALL_DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/MotorDriveLayer"
)
install(
  EXPORT MotorDriveLayerTargets
  FILE MotorDriveLayerTargets.cmake
  NAMESPACE motorbridge::
  DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/MotorDriveLayer"
)

set(MOTOR_DM_DEVICE_INSTALL_DIR "${CMAKE_INSTALL_LIBDIR}/dm_device")
if(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND
   CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)$")
  install(
    FILES third_party/dm_device/v1.1.0/linux/arm64/libdm_device.so
    DESTINATION "${MOTOR_DM_DEVICE_INSTALL_DIR}/v1.1.0"
  )
elseif(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND
       CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|amd64|AMD64)$")
  install(
    FILES third_party/dm_device/v1.0.0/linux/libdm_device.so
    DESTINATION "${MOTOR_DM_DEVICE_INSTALL_DIR}/v1.0.0"
  )
  install(
    FILES third_party/dm_device/v1.1.0/linux/x86_64/libdm_device.so
    DESTINATION "${MOTOR_DM_DEVICE_INSTALL_DIR}/v1.1.0"
  )
  install(
    FILES third_party/libstdcxx/linux/x86_64/libstdc++.so.6
    DESTINATION "${MOTOR_DM_DEVICE_INSTALL_DIR}"
  )
elseif(APPLE AND CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)$")
  install(
    FILES third_party/dm_device/v1.1.0/macos/arm64/libdm_device.dylib
    DESTINATION "${MOTOR_DM_DEVICE_INSTALL_DIR}/v1.1.0"
  )
elseif(APPLE)
  install(
    FILES third_party/dm_device/v1.1.0/macos/x86_64/libdm_device.dylib
    DESTINATION "${MOTOR_DM_DEVICE_INSTALL_DIR}/v1.1.0"
  )
elseif(WIN32)
  install(
    FILES third_party/dm_device/v1.1.0/windows/msvc/dm_device.dll
    DESTINATION "${CMAKE_INSTALL_BINDIR}/dm_device/v1.1.0"
  )
endif()
install(
  FILES LICENSE bindings/python/LICENSES/DM_DEVICE_RUNTIME_NOTICE
  DESTINATION "${CMAKE_INSTALL_DATADIR}/licenses/MotorDriveLayer"
)
install(
  FILES
    "${CMAKE_CURRENT_BINARY_DIR}/MotorDriveLayerConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/MotorDriveLayerConfigVersion.cmake"
  DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/MotorDriveLayer"
)
