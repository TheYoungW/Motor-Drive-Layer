cmake_minimum_required(VERSION 3.16)
project(motorbridge_native VERSION 0.7.9 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(CTest)

add_subdirectory(cpp_damiao)
add_subdirectory(articore_runtime)
