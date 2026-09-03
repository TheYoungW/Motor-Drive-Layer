add_library(articore_dds STATIC
  src/lease.cpp
  src/protocol.cpp
)
add_library(articore::dds ALIAS articore_dds)
target_compile_features(articore_dds PUBLIC cxx_std_17)
target_include_directories(articore_dds PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>)
target_link_libraries(articore_dds PUBLIC articore::runtime)
set_target_properties(articore_dds PROPERTIES
  POSITION_INDEPENDENT_CODE ON
  CXX_VISIBILITY_PRESET hidden
  VISIBILITY_INLINES_HIDDEN YES)

if(ARTICORE_ENABLE_DDS)
  find_package(CycloneDDS 11.0.1 EXACT CONFIG REQUIRED)
  if(CMAKE_CROSSCOMPILING)
    set(ARTICORE_HOST_IDLC "" CACHE FILEPATH
      "Host-native Cyclone DDS 11.0.1 idlc executable")
    if(NOT ARTICORE_HOST_IDLC)
      message(FATAL_ERROR
        "Cross builds require -DARTICORE_HOST_IDLC=/path/to/host/idlc")
    endif()
    get_filename_component(ARTICORE_HOST_IDLC_BIN_DIR
      "${ARTICORE_HOST_IDLC}" DIRECTORY)
    get_filename_component(ARTICORE_HOST_CYCLONEDDS_PREFIX
      "${ARTICORE_HOST_IDLC_BIN_DIR}" DIRECTORY)
    find_library(ARTICORE_HOST_IDLC_BACKEND
      NAMES cycloneddsidlc
      PATHS "${ARTICORE_HOST_CYCLONEDDS_PREFIX}/lib"
      NO_DEFAULT_PATH)
    if(NOT ARTICORE_HOST_IDLC_BACKEND)
      message(FATAL_ERROR
        "Host idlc backend libcycloneddsidlc was not found beside ARTICORE_HOST_IDLC")
    endif()
    # An ARM Cyclone installation built with BUILD_IDLC=OFF intentionally has
    # no executable target. Define the imported host tool while keeping ddsc
    # and all headers supplied by the ARM64 package.
    if(NOT TARGET CycloneDDS::idlc)
      add_executable(CycloneDDS::idlc IMPORTED GLOBAL)
    endif()
    set_property(TARGET CycloneDDS::idlc PROPERTY
      IMPORTED_LOCATION "${ARTICORE_HOST_IDLC}")
    set_property(TARGET CycloneDDS::idlc PROPERTY
      IMPORTED_LOCATION_RELEASE "${ARTICORE_HOST_IDLC}")
    # Cyclone's generation helper selects the imported executable when this
    # build-only flag is true. The backend is a host library, never linked to
    # the target service.
    set(BUILD_IDLC ON)
    set(_idlc_backend "${ARTICORE_HOST_IDLC_BACKEND}")
  endif()
  idlc_generate(TARGET articore_dds_types
    FILES ${CMAKE_CURRENT_LIST_DIR}/idl/articore_protocol.idl
    WARNINGS no-implicit-extensibility)
  target_sources(articore_dds PRIVATE src/cyclone_service.cpp)
  target_compile_definitions(articore_dds PRIVATE
    ARTICORE_SERVICE_VERSION="${PROJECT_VERSION}")
  target_link_libraries(articore_dds PRIVATE
    articore_dds_types CycloneDDS::ddsc)

  add_executable(articore-runtime-service src/main.cpp src/config.cpp)
  target_link_libraries(articore-runtime-service PRIVATE articore::dds)
  set_target_properties(articore-runtime-service PROPERTIES
    INSTALL_RPATH "$ORIGIN/../lib/articore")
  install(TARGETS articore-runtime-service
          RUNTIME DESTINATION usr/bin)
  install(FILES $<TARGET_FILE:CycloneDDS::ddsc>
                $<TARGET_SONAME_FILE:CycloneDDS::ddsc>
          DESTINATION usr/lib/articore)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(articore_dds PRIVATE -Wall -Wextra -Wpedantic)
endif()

if(BUILD_TESTING)
  add_executable(articore_dds_protocol_test tests/protocol_test.cpp)
  target_link_libraries(articore_dds_protocol_test PRIVATE articore::dds)
  add_test(NAME dds_protocol_and_lease COMMAND articore_dds_protocol_test)

  if(ARTICORE_ENABLE_DDS)
    add_executable(articore_dds_integration_test
      tests/cyclone_integration_test.cpp src/config.cpp)
    target_link_libraries(articore_dds_integration_test PRIVATE
      articore::dds articore_dds_types CycloneDDS::ddsc)
    add_test(NAME dds_two_participant_loopback
             COMMAND articore_dds_integration_test)
  endif()

  add_test(NAME architecture_layers
    COMMAND ${CMAKE_COMMAND}
      -DROOT=${CMAKE_SOURCE_DIR}
      -P ${CMAKE_CURRENT_LIST_DIR}/tests/check_layers.cmake)
endif()
