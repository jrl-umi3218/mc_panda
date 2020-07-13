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

add_robot(panda src/panda.cpp src/panda.h)
target_include_directories(panda PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/include")

option(DISABLE_TESTS "Disable unit tests" OFF)
if(NOT ${DISABLE_TESTS})
  enable_testing()
  add_subdirectory(tests)
endif()
