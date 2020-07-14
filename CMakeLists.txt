cmake_minimum_required(VERSION 3.1)

project(mc-panda)

set(CMAKE_CXX_STANDARD 11)

#find_package(panda_description REQUIRED)
find_package(mc_rtc REQUIRED)
find_description_package(franka_gazebo REQUIRED)

#if("${FRANKA_GAZEBO_INSTALL_PREFIX}" STREQUAL "")
#  if("${FRANKA_GAZEBO_SOURCE_PREFIX}" STREQUAL "")
#    message(FATAL_ERROR "Your franka_gazebo package does not define where to find the data")
#  endif()
#  set(FRANKA_GAZEBO_PATH "${FRANKA_GAZEBO_SOURCE_PREFIX}")
#else()
#  set(FRANKA_GAZEBO_PATH "${FRANKA_GAZEBO_INSTALL_PREFIX}")
#endif()

message("FRANKA_GAZEBO_PATH: ${FRANKA_GAZEBO_PATH}")
configure_file(src/config.h.in "${CMAKE_CURRENT_BINARY_DIR}/include/config.h")

# Panda custom devices library
set(devices_HDR
  src/devices/Pump.h
)
set(devices_SRC
  src/devices/Pump.cpp
)

add_library(mc_panda_devices SHARED ${devices_SRC} ${devices_HDR})
add_library(mc_panda::devices ALIAS mc_panda_devices)
set_target_properties(mc_panda_devices PROPERTIES EXPORT_NAME devices)
target_include_directories(mc_panda_devices PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(mc_panda_devices PUBLIC mc_rtc::mc_rbdyn)
install(FILES ${devices_HDR} DESTINATION include/mc_panda/devices)
install(TARGETS mc_panda_devices
  EXPORT mc_pandaTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Panda custom tasks library
#...

# Panda custom constraints library
#...

# Panda robot module library
add_robot(panda src/panda.cpp src/panda.h)
target_include_directories(panda PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/include")
#target_link_libraries(panda PRIVATE mc_panda::tasks)
target_link_libraries(panda PRIVATE mc_panda::devices)
#target_link_libraries(panda PRIVATE mc_panda::constraints)

install(EXPORT mc_pandaTargets
  FILE mc_pandaTargets.cmake
  NAMESPACE mc_panda::
  DESTINATION lib/cmake/mc_panda)

# Tests
option(DISABLE_TESTS "Disable unit tests" OFF)
if(NOT ${DISABLE_TESTS})
  enable_testing()
  add_subdirectory(tests)
endif()
