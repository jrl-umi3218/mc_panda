configure_file(${PROJECT_SOURCE_DIR}/include/mc_panda/config.in.h "${CMAKE_BINARY_DIR}/include/mc_panda/config.h")

set(MC_PANDA_PREFIX "MC_PANDA")
configure_file(${PROJECT_SOURCE_DIR}/include/mc_panda/api.in.h "${CMAKE_BINARY_DIR}/include/mc_panda/api.h")

set(MC_PANDA_PREFIX "MC_PANDA_DEVICES")
configure_file(${PROJECT_SOURCE_DIR}/include/mc_panda/api.in.h "${CMAKE_BINARY_DIR}/include/mc_panda/devices/api.h")

# Panda custom devices library
set(devices_HDR
  ${PROJECT_SOURCE_DIR}/include/mc_panda/devices/Robot.h
  ${PROJECT_SOURCE_DIR}/include/mc_panda/devices/Pump.h
  "${CMAKE_BINARY_DIR}/include/mc_panda/devices/api.h"
)
set(devices_SRC
  devices/Robot.cpp
  devices/Pump.cpp
)

add_library(mc_panda_devices SHARED ${devices_SRC} ${devices_HDR})
add_library(mc_panda::devices ALIAS mc_panda_devices)
set_target_properties(mc_panda_devices PROPERTIES EXPORT_NAME devices)
set_target_properties(mc_panda_devices PROPERTIES SOVERSION ${PROJECT_VERSION_MAJOR} VERSION ${PROJECT_VERSION})
target_include_directories(mc_panda_devices 
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
target_link_libraries(mc_panda_devices PUBLIC mc_rtc::mc_rbdyn Franka::Franka)
target_compile_definitions(mc_panda_devices PRIVATE -DMC_PANDA_DEVICES_EXPORTS)
install(FILES ${devices_HDR} DESTINATION include/mc_panda/devices)
install(TARGETS mc_panda_devices
  EXPORT mc_pandaTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Panda robot module library
set(panda_HDR
  ${PROJECT_SOURCE_DIR}/include/mc_panda/panda.h
  "${CMAKE_BINARY_DIR}/include/mc_panda/api.h"
  "${CMAKE_BINARY_DIR}/include/mc_panda/config.h"
)

set(panda_SRC
  panda.cpp
)

add_library(mc_panda SHARED panda.cpp ${panda_SRC} ${panda_HDR})
add_library(mc_panda::panda ALIAS mc_panda)
set_target_properties(mc_panda PROPERTIES EXPORT_NAME panda)
set_target_properties(mc_panda PROPERTIES SOVERSION ${PROJECT_VERSION_MAJOR} VERSION ${PROJECT_VERSION})
target_include_directories(mc_panda PUBLIC $<INSTALL_INTERFACE:include> $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>)
target_include_directories(mc_panda INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)
target_link_libraries(mc_panda PUBLIC mc_panda::devices)
target_compile_definitions(mc_panda PRIVATE -DMC_PANDA_EXPORTS)
install(FILES ${panda_HDR} DESTINATION include/mc_panda)
install(TARGETS mc_panda
  EXPORT mc_pandaTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

add_robot(panda module.cpp "")
target_link_libraries(panda PRIVATE mc_panda::panda)

add_robot(panda_pump panda_pump.cpp "")
target_link_libraries(panda_pump PRIVATE mc_panda::devices)

# Panda custom states library
add_subdirectory(states)

install(EXPORT mc_pandaTargets
  FILE mc_pandaTargets.cmake
  NAMESPACE mc_panda::
  DESTINATION lib/cmake/mc_panda)
