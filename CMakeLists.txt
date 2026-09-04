cmake_minimum_required(VERSION 3.20)
project(motorbridge VERSION 1.2.7 LANGUAGES C CXX)

if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
  message(FATAL_ERROR "articore-runtime-service supports Linux SocketCAN-FD only")
endif()

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(ARTICORE_ENABLE_DDS "Build the Cyclone DDS transport and service" ON)
option(ARTICORE_BUILD_DEPLOYMENT "Install RK3588 configuration and systemd units" ON)

include(CTest)
include(GNUInstallDirs)

add_subdirectory(motor)
add_subdirectory(runtime)
add_subdirectory(dds)

if(ARTICORE_BUILD_DEPLOYMENT AND ARTICORE_ENABLE_DDS)
  install(FILES deploy/runtime-service.conf deploy/cyclonedds.xml
          DESTINATION etc/articore)
  install(FILES deploy/can.conf
          DESTINATION etc/articore)
  install(PROGRAMS deploy/articore-can-init deploy/articore-can-hotplug
                   deploy/articore-wait-dds-network
          DESTINATION usr/libexec/articore)
  install(FILES deploy/articore-can.service deploy/articore-runtime.service
                deploy/articore-can-hotplug.service
          DESTINATION usr/lib/systemd/system)
  install(FILES deploy/99-articore-can-hotplug.rules
          DESTINATION usr/lib/udev/rules.d)
  install(FILES deploy/articore-dds-firewall.nft
          DESTINATION usr/share/articore)
  install(FILES deploy/build-info.txt
          DESTINATION usr/share/articore)
  install(FILES dds/idl/articore_protocol.idl
          DESTINATION usr/share/articore/idl)
endif()

set(CPACK_PACKAGE_NAME "articore-runtime-service")
set(CPACK_PACKAGE_VERSION ${PROJECT_VERSION})
set(CPACK_PACKAGE_CONTACT "Articore Robotics")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY
    "RK3588 dual-arm Runtime service with Cyclone DDS")
set(CPACK_DEBIAN_PACKAGE_SECTION "utils")
set(CPACK_INSTALL_DEFAULT_DIRECTORY_PERMISSIONS
    OWNER_READ OWNER_WRITE OWNER_EXECUTE
    GROUP_READ GROUP_EXECUTE
    WORLD_READ WORLD_EXECUTE)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)$")
  set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "arm64")
else()
  set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "amd64")
endif()
set(CPACK_PACKAGE_FILE_NAME
    "${CPACK_PACKAGE_NAME}_${CPACK_PACKAGE_VERSION}_${CPACK_DEBIAN_PACKAGE_ARCHITECTURE}")
set(CPACK_DEBIAN_PACKAGE_DEPENDS "adduser, iproute2, systemd, udev")
if(CMAKE_CROSSCOMPILING)
  # Host dpkg-shlibdeps cannot resolve target libc/libstdc++ from an ARM64
  # cross toolchain. These are the only non-private runtime dependencies for
  # the Ubuntu 22.04 target; libddsc is shipped inside this package.
  set(CPACK_DEBIAN_PACKAGE_SHLIBDEPS OFF)
  string(APPEND CPACK_DEBIAN_PACKAGE_DEPENDS
      ", libc6 (>= 2.35), libgcc-s1 (>= 3.0), libstdc++6 (>= 11)")
else()
  set(CPACK_DEBIAN_PACKAGE_SHLIBDEPS ON)
endif()
set(CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA
    "${CMAKE_CURRENT_SOURCE_DIR}/deploy/debian/postinst;${CMAKE_CURRENT_SOURCE_DIR}/deploy/debian/prerm;${CMAKE_CURRENT_SOURCE_DIR}/deploy/debian/postrm;${CMAKE_CURRENT_SOURCE_DIR}/deploy/debian/conffiles")
set(CPACK_PACKAGING_INSTALL_PREFIX "/")
set(CPACK_GENERATOR "DEB")
include(CPack)

if(BUILD_TESTING)
  add_test(NAME deployment_scripts
    COMMAND sh ${CMAKE_CURRENT_SOURCE_DIR}/tests/deployment_scripts_test.sh
               ${CMAKE_CURRENT_SOURCE_DIR})
endif()
