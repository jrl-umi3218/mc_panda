cmake_minimum_required(VERSION 3.1)
project(mc_panda LANGUAGES CXX VERSION 1.0.0)
enable_testing()
set(CMAKE_CXX_STANDARD 11)

find_program(XACRO xacro)
if("${XACRO}" STREQUAL "XACRO-NOTFOUND")
  message(FATAL_ERROR "Could not find xacro in your environment")
endif()

find_package(mc_rtc 1.4.0 REQUIRED)
find_package(Franka REQUIRED)

if(NOT EXISTS "${CMAKE_INSTALL_PREFIX}/share/panda_cnoid/panda_default.urdf")
  message(FATAL_ERROR "You need to install panda_cnoid before this project")
endif()
set(PANDA_DESCRIPTION_PATH "${CMAKE_INSTALL_PREFIX}/share/panda_cnoid")

include(CMakePackageConfigHelpers)
set(VERSION_CONFIG "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake")
set(PROJECT_CONFIG "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake")
set(CONFIG_INSTALL_DIR "lib/cmake/${PROJECT_NAME}")
write_basic_package_version_file(
  ${VERSION_CONFIG}
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)
configure_package_config_file("${CMAKE_CURRENT_SOURCE_DIR}/cmake/Config.cmake.in" "${PROJECT_CONFIG}" INSTALL_DESTINATION "${CONFIG_INSTALL_DIR}")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake" "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake" DESTINATION "${CONFIG_INSTALL_DIR}")

configure_file(src/config.h.in "${CMAKE_CURRENT_BINARY_DIR}/include/config.h")

# Panda custom devices library
set(devices_HDR
  src/devices/PandaDevice.h
  src/devices/Pump.h
)
set(devices_SRC
  src/devices/PandaDevice.cpp
  src/devices/Pump.cpp
)

add_library(mc_panda_devices SHARED ${devices_SRC} ${devices_HDR})
add_library(mc_panda::devices ALIAS mc_panda_devices)
set_target_properties(mc_panda_devices PROPERTIES EXPORT_NAME devices)
target_include_directories(mc_panda_devices PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(mc_panda_devices PUBLIC mc_rtc::mc_rbdyn Franka::Franka)
install(FILES ${devices_HDR} DESTINATION include/mc_panda/devices)
install(TARGETS mc_panda_devices
  EXPORT mc_pandaTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Panda custom states library
add_subdirectory(src/states)

# Panda robot module library
add_robot(panda src/panda.cpp src/panda.h)
target_include_directories(panda PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/include")
target_link_libraries(panda PRIVATE mc_panda::devices)

install(EXPORT mc_pandaTargets
  FILE mc_pandaTargets.cmake
  NAMESPACE mc_panda::
  DESTINATION lib/cmake/mc_panda)
