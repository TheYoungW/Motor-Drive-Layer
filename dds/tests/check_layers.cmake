file(GLOB_RECURSE MOTOR_FILES
  "${ROOT}/motor/*.hpp" "${ROOT}/motor/*.cpp")
file(GLOB_RECURSE RUNTIME_FILES
  "${ROOT}/runtime/*.hpp" "${ROOT}/runtime/*.cpp")
foreach(FILE_PATH IN LISTS MOTOR_FILES)
  file(READ "${FILE_PATH}" CONTENT)
  if(CONTENT MATCHES "articore/(runtime|dds)" OR CONTENT MATCHES "dds/")
    message(FATAL_ERROR "motor layer has an upward dependency: ${FILE_PATH}")
  endif()
endforeach()
foreach(FILE_PATH IN LISTS RUNTIME_FILES)
  file(READ "${FILE_PATH}" CONTENT)
  if(CONTENT MATCHES "articore/dds" OR CONTENT MATCHES "dds/")
    message(FATAL_ERROR "runtime layer depends on dds: ${FILE_PATH}")
  endif()
  if(CONTENT MATCHES "runtime_bridge" OR
     CONTENT MATCHES "robot_model_bridge")
    message(FATAL_ERROR "legacy Runtime bridge returned: ${FILE_PATH}")
  endif()
endforeach()

if(EXISTS "${ROOT}/runtime/include/articore/detail/runtime_bridge.hpp" OR
   EXISTS "${ROOT}/runtime/include/articore/detail/runtime_bridge_client.hpp" OR
   EXISTS "${ROOT}/runtime/src/runtime_facade_bridge.cpp")
  message(FATAL_ERROR "legacy Runtime bridge files must not be restored")
endif()
