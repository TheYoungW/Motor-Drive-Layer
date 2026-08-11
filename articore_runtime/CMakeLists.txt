cmake_minimum_required(VERSION 3.16)
project(articore_runtime LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(articore_runtime_core STATIC src/runtime.cpp)
add_library(articore::runtime_core ALIAS articore_runtime_core)
target_include_directories(articore_runtime_core PUBLIC include src)
target_compile_features(articore_runtime_core PUBLIC cxx_std_17)
set_target_properties(
  articore_runtime_core PROPERTIES
  POSITION_INDEPENDENT_CODE ON
  CXX_VISIBILITY_PRESET hidden
  VISIBILITY_INLINES_HIDDEN YES
)

add_library(articore_runtime SHARED src/runtime_abi.cpp)
add_library(articore::runtime ALIAS articore_runtime)
target_link_libraries(articore_runtime PRIVATE articore::runtime_core)
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
endif()
set_target_properties(
  articore_runtime PROPERTIES
  WINDOWS_EXPORT_ALL_SYMBOLS ON
  OUTPUT_NAME articore_runtime
  CXX_VISIBILITY_PRESET hidden
  VISIBILITY_INLINES_HIDDEN YES
)

if(MSVC)
  target_compile_options(articore_runtime_core PRIVATE /W4)
else()
  target_compile_options(articore_runtime_core PRIVATE -Wall -Wextra -Wpedantic)
endif()

include(GNUInstallDirs)
install(
  TARGETS articore_runtime
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

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
endif()
