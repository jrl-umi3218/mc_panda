cmake_minimum_required(VERSION 3.22)

project(mc_panda LANGUAGES CXX VERSION 2.0.0)
enable_testing()
set(CMAKE_CXX_STANDARD 17)

find_package(mc_rtc REQUIRED)
find_package(Franka REQUIRED)

set(PANDA_DESCRIPTION_PATH "${CMAKE_INSTALL_PREFIX}/share/mc_panda" CACHE STRING "Path to panda description")
set(FR1_DESCRIPTION_PATH "${CMAKE_INSTALL_PREFIX}/share/mc_panda/fr1" CACHE STRING "Path to panda description")
set(FR3_DESCRIPTION_PATH "${CMAKE_INSTALL_PREFIX}/share/mc_panda/fr3" CACHE STRING "Path to panda description")
set(TOOLS_DESCRIPTION_PATH "${CMAKE_INSTALL_PREFIX}/share/mc_panda/tools" CACHE STRING "Path to panda tools (pump, etc) description")
add_subdirectory(data)
add_subdirectory(src)
add_subdirectory(cmake)

# FIXME: testing depends on installed files, robot aliases, etc
# To test the robot module at build time, we would need to have the whole module in the build folder
# which is not trivial to achieve / not very representative of the final installed module either
# if(BUILD_TESTING)
#   add_subdirectory(tests)
# endif()

install(FILES etc/ForceSensorCalibration/panda_default.yaml DESTINATION ${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/ForceSensorCalibration/)
