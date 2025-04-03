cmake_minimum_required(VERSION 3.5)

# Disable in-source builds
if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
  message(FATAL_ERROR "This project doesn't support in-source builds")
endif()

project(mc_panda LANGUAGES CXX VERSION 1.1.1)
enable_testing()
set(CMAKE_CXX_STANDARD 11)

find_package(mc_rtc REQUIRED)
find_package(Franka REQUIRED)

add_subdirectory(data)
add_subdirectory(src)
add_subdirectory(cmake)

if(BUILD_TESTING)
  add_subdirectory(tests)
endif()

install(FILES etc/ForceSensorCalibration/panda_default.yaml DESTINATION ${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/ForceSensorCalibration/)
