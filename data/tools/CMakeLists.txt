set(MODULE_INSTALL_LOCATION "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/tools")

macro(add_yaml_module NAME)
  configure_file(urdf/${NAME}.in.urdf "${CMAKE_CURRENT_BINARY_DIR}/urdf/${NAME}.urdf")
  configure_file(yaml/${NAME}.in.yaml "${CMAKE_CURRENT_BINARY_DIR}/yaml/${NAME}.yaml")
# Configure the alias accordingly
  configure_file(alias/${NAME}.in.yaml "${CMAKE_CURRENT_BINARY_DIR}/alias/${NAME}.yaml")
# Install the alias
  install(FILES "${CMAKE_CURRENT_BINARY_DIR}/alias/${NAME}.yaml" DESTINATION "${MC_ROBOTS_ALIASES_DIRECTORY}")
  install(FILES "${CMAKE_CURRENT_BINARY_DIR}/urdf/${NAME}.urdf" DESTINATION "${MODULE_INSTALL_LOCATION}/urdf")
  install(FILES "${CMAKE_CURRENT_BINARY_DIR}/yaml/${NAME}.yaml" DESTINATION "${MODULE_INSTALL_LOCATION}/yaml")
  install(DIRECTORY rsdf/${NAME} DESTINATION "${MODULE_INSTALL_LOCATION}/rsdf" FILES_MATCHING PATTERN "*.rsdf")
  install(DIRECTORY convex/${NAME} DESTINATION "${MODULE_INSTALL_LOCATION}/convex" FILES_MATCHING PATTERN "*.txt")
  install(DIRECTORY meshes/${NAME} DESTINATION "${MODULE_INSTALL_LOCATION}/meshes"
    FILES_MATCHING PATTERN "*.stl" PATTERN "*.STL" PATTERN "*.dae" PATTERN "*.DAE"
  )
endmacro()
add_yaml_module(panda_hand)
add_yaml_module(panda_pump)
add_yaml_module(panda_foot)
