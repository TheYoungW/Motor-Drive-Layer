cmake_minimum_required(VERSION 3.16)
project(motorbridge_native VERSION 0.12.2 LANGUAGES CXX)

if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
  message(FATAL_ERROR "The Yunyi Runtime supports Linux SocketCAN-FD only")
endif()

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

install(FILES LICENSE DESTINATION "${CMAKE_INSTALL_DATADIR}/licenses/MotorDriveLayer")
install(
  FILES
    "${CMAKE_CURRENT_BINARY_DIR}/MotorDriveLayerConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/MotorDriveLayerConfigVersion.cmake"
  DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/MotorDriveLayer"
)
