set(MODULE_INSTALL_LOCATION "${TOOLS_DESCRIPTION_PATH}")

macro(add_yaml_module NAME)
  configure_file(urdf/${NAME}.in.urdf "${CMAKE_CURRENT_BINARY_DIR}/urdf/${NAME}.urdf")
  install(FILES "${CMAKE_CURRENT_BINARY_DIR}/urdf/${NAME}.urdf" DESTINATION "${MODULE_INSTALL_LOCATION}/urdf")

  if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/yaml/${NAME}.in.yaml")
    configure_file(yaml/${NAME}.in.yaml "${CMAKE_CURRENT_BINARY_DIR}/yaml/${NAME}.yaml")
    install(FILES "${CMAKE_CURRENT_BINARY_DIR}/yaml/${NAME}.yaml" DESTINATION "${MODULE_INSTALL_LOCATION}/yaml")
  endif()

  if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/alias/${NAME}.in.yaml")
    configure_file(alias/${NAME}.in.yaml "${CMAKE_CURRENT_BINARY_DIR}/alias/${NAME}.yaml")
    install(FILES "${CMAKE_CURRENT_BINARY_DIR}/alias/${NAME}.yaml" DESTINATION "${MC_ROBOTS_ALIASES_DIRECTORY}")
  endif()

  install(DIRECTORY rsdf/${NAME} DESTINATION "${MODULE_INSTALL_LOCATION}/rsdf" FILES_MATCHING PATTERN "*.rsdf")
  install(DIRECTORY convex/${NAME} DESTINATION "${MODULE_INSTALL_LOCATION}/convex" FILES_MATCHING PATTERN "*.txt")
  install(DIRECTORY meshes/${NAME} DESTINATION "${MODULE_INSTALL_LOCATION}/meshes"
    FILES_MATCHING PATTERN "*.stl" PATTERN "*.STL" PATTERN "*.dae" PATTERN "*.DAE"
  )
endmacro()
add_yaml_module(panda_hand)
add_yaml_module(panda_foot)
# panda_pump is a c++ module, we only install urdf/rsdf/convex but no alias
add_yaml_module(panda_pump)
