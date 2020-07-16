cmake_minimum_required(VERSION 3.1)
project(mc_panda LANGUAGES CXX VERSION 1.0.0)
enable_testing()
set(CMAKE_CXX_STANDARD 11)

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

find_package(Boost REQUIRED COMPONENTS timer serialization filesystem)
find_package(mc_rtc 1.4.0 REQUIRED)

#find_description_package(panda_description)
#set(PANDA_DESCRIPTION_PATH "/usr/local") #TODO
#message("PANDA_DESCRIPTION_PATH: ${PANDA_DESCRIPTION_PATH}")
#taken from panda_cnoid
#find_description_package(franka_gazebo REQUIRED) 
#message("FRANKA_GAZEBO_PATH: ${FRANKA_GAZEBO_PATH}")
#taken from mc_franka
#find_package(Franka REQUIRED)
#get_target_property(Franka_INCLUDE_DIRS ${Franka_LIBRARIES} INTERFACE_INCLUDE_DIRECTORIES)
#include_directories(${Franka_INCLUDE_DIRS}
#    ${CMAKE_SOURCE_DIR}/../libfranka/src
#    ${CMAKE_SOURCE_DIR}/../libfranka/common/include
#)
#message(STATUS "Franka_LIBRARIES: ${Franka_LIBRARIES}")
#message(STATUS "Franka_DIR: ${Franka_DIR}")
#message(STATUS "Franka_INCLUDE_DIRS: ${Franka_INCLUDE_DIRS}")
#message(STATUS "CMAKE_SOURCE_DIR: ${CMAKE_SOURCE_DIR}")
#link_directories(${Franka_LIBRARY_DIRS})
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
#set(tasks_HDR
#  src/tasks/CoMRelativeBodyTask.h
#)
#set(tasks_SRC
#  src/tasks/CoMRelativeBodyTask.cpp
#)
#add_library(mc_panda_tasks SHARED ${tasks_SRC} ${tasks_HDR})
#add_library(mc_panda::tasks ALIAS mc_panda_tasks)
#set_target_properties(mc_panda_tasks PROPERTIES EXPORT_NAME tasks)
#target_include_directories(mc_panda_tasks PUBLIC
#  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
#  $<INSTALL_INTERFACE:include>)
#target_link_libraries(mc_panda_tasks PUBLIC mc_rtc::mc_tasks)
#install(FILES ${tasks_HDR} DESTINATION include/mc_panda/tasks)
#install(TARGETS mc_panda_tasks
#  EXPORT mc_pandaTargets
#  ARCHIVE DESTINATION lib
#  LIBRARY DESTINATION lib
#  RUNTIME DESTINATION bin)

# Panda custom constraints library
#set(constraints_HDR
#  src/constraints/BoundedAccelerationConstr.h
#)
#set(constraints_SRC
#  src/constraints/BoundedAccelerationConstr.cpp
#)
#add_library(mc_panda_constraints SHARED ${constraints_SRC} ${constraints_HDR})
#add_library(mc_panda::constraints ALIAS mc_panda_constraints)
#set_target_properties(mc_panda_constraints PROPERTIES EXPORT_NAME constraints)
#target_include_directories(mc_panda_constraints PUBLIC
#  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
#  $<INSTALL_INTERFACE:include>)
#target_link_libraries(mc_panda_constraints PUBLIC mc_rtc::mc_solver)
#install(FILES ${constraints_HDR} DESTINATION include/mc_panda/constraints)
#install(TARGETS mc_panda_constraints
#  EXPORT mc_pandaTargets
#  ARCHIVE DESTINATION lib
#  LIBRARY DESTINATION lib
#  RUNTIME DESTINATION bin)

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
