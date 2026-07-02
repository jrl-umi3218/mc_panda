set(URDF_DESTINATION "${FR1_DESCRIPTION_PATH}/urdf")
set(RSDF_DESTINATION "${FR1_DESCRIPTION_PATH}/rsdf")
set(CONVEX_DESTINATION "${FR1_DESCRIPTION_PATH}/convex")
set(MESHES_DESTINATION "${FR1_DESCRIPTION_PATH}/meshes")

configure_file(urdf/panda_default.in.urdf "${CMAKE_CURRENT_BINARY_DIR}/urdf/panda_default.urdf" @ONLY)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/urdf/panda_default.urdf" DESTINATION "${URDF_DESTINATION}")

install(DIRECTORY meshes/visual DESTINATION "${MESHES_DESTINATION}" FILES_MATCHING PATTERN "*.dae")

# Install extra meshes
install(FILES meshes/foot.stl meshes/pump.dae DESTINATION "${MESHES_DESTINATION}")

# Install convex
install(DIRECTORY convex DESTINATION "${FR1_DESCRIPTION_PATH}" FILES_MATCHING PATTERN "*-ch.txt")

# Install rsdf
install(DIRECTORY rsdf DESTINATION "${FR1_DESCRIPTION_PATH}/" FILES_MATCHING PATTERN "*.rsdf")

find_package(choreonoid QUIET)
if(choreonoid_FOUND)
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(OPENHRP REQUIRED openhrp3.1)
  pkg_check_modules(HRPSYS_BASE REQUIRED hrpsys-base)
  set(MODEL_DESTINATION "${OPENHRP_PREFIX}/share/OpenHRP-3.1/robot/panda")
  # Install the VRML model
  install(DIRECTORY model/panda/ DESTINATION ${MODEL_DESTINATION} FILES_MATCHING PATTERN "*.wrl")
endif()

macro(GENERATE_CNOID_PROJECT PROJECT_FOLDER)
  set(PROJECT_DESTINATION "${HRPSYS_BASE_PREFIX}/share/hrpsys/samples/${PROJECT_FOLDER}")
  set(MODEL_OUT "${MODEL_DESTINATION}/${PROJECT_FOLDER}.wrl")
  file(READ cnoid/${PROJECT_FOLDER}/joint_positions.cnoid JOINT_POSITIONS)
  string(STRIP "${JOINT_POSITIONS}" JOINT_POSITIONS)
  set(USE_UDP False)
  set(SIM_SCRIPT sim_mc.py)
  set(ROBOT_NAME "${PROJECT_FOLDER}")
  set(SENSOR_PORT_A "LeftHandForceSensor")
  set(CONF_FILE_NAME "Virtual-${PROJECT_FOLDER}-RTC.conf")
  install(FILES "cnoid/${PROJECT_FOLDER}/Virtual-${PROJECT_FOLDER}-RTC.conf" DESTINATION "${CHOREONOID_PLUGIN_DIR}/rtc/")
  set(PROJECT_FILES
    cnoid/common/clear-omninames.sh
    cnoid/common/robot.conf
    cnoid/common/rtc.conf.choreonoid
    cnoid/common/sim_mc.cnoid
    cnoid/common/sim_mc_withTable.cnoid
    cnoid/${PROJECT_FOLDER}/sim_mc.py
    cnoid/${PROJECT_FOLDER}/PDcontroller.conf.choreonoid
    cnoid/${PROJECT_FOLDER}/PDgains_sim.dat
  )
  foreach(F ${PROJECT_FILES})
    get_filename_component(F_NAME "${F}" NAME)
    configure_file(${F} "${CMAKE_BINARY_DIR}/cnoid/${PROJECT_FOLDER}/${F_NAME}" @ONLY)
  endforeach()
  set(USE_UDP True)
  set(SIM_SCRIPT sim_mc_udp.py)
  set(SENSOR_PORT_A "lhsensor")
  configure_file(cnoid/${PROJECT_FOLDER}/sim_mc.py "${CMAKE_BINARY_DIR}/cnoid/${PROJECT_FOLDER}/${SIM_SCRIPT}" @ONLY)
  configure_file(cnoid/common/sim_mc.cnoid "${CMAKE_BINARY_DIR}/cnoid/${PROJECT_FOLDER}/sim_mc_udp.cnoid" @ONLY)
  configure_file(cnoid/common/sim_mc_withTable.cnoid "${CMAKE_BINARY_DIR}/cnoid/${PROJECT_FOLDER}/sim_mc_udp.cnoid" @ONLY)
  install(DIRECTORY "${CMAKE_BINARY_DIR}/cnoid/${PROJECT_FOLDER}/" DESTINATION "${PROJECT_DESTINATION}")
  install(PROGRAMS cnoid/common/clear-omninames.sh DESTINATION "${PROJECT_DESTINATION}")
endmacro()

if(choreonoid_FOUND)
  generate_cnoid_project(panda_default)
  generate_cnoid_project(panda_hand)
  generate_cnoid_project(panda_foot)
  generate_cnoid_project(panda_pump)
endif()
