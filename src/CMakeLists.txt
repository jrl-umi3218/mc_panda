configure_file(config.h.in "${CMAKE_CURRENT_BINARY_DIR}/include/config.h")

# Panda custom devices library
set(devices_HDR
  devices/Robot.h
  devices/Pump.h
)
set(devices_SRC
  devices/Robot.cpp
  devices/Pump.cpp
)

add_library(mc_panda_devices SHARED ${devices_SRC} ${devices_HDR})
add_library(mc_panda::devices ALIAS mc_panda_devices)
set_target_properties(mc_panda_devices PROPERTIES EXPORT_NAME devices)
target_include_directories(mc_panda_devices PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>$<INSTALL_INTERFACE:include>)
target_link_libraries(mc_panda_devices PUBLIC mc_rtc::mc_rbdyn Franka::Franka)
install(FILES ${devices_HDR} DESTINATION include/mc_panda/devices)
install(TARGETS mc_panda_devices
  EXPORT mc_pandaTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Panda robot module library
add_robot(panda panda.cpp panda.h)
target_include_directories(panda PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/include")
target_link_libraries(panda PRIVATE mc_panda::devices)

# Panda custom states library
add_subdirectory(states)

install(EXPORT mc_pandaTargets
  FILE mc_pandaTargets.cmake
  NAMESPACE mc_panda::
  DESTINATION lib/cmake/mc_panda)
