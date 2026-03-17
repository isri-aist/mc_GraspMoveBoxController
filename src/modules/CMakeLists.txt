set(MODULE_INSTALL_LOCATION "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}")

macro(add_yaml_module NAME)
  configure_file(urdf/${NAME}.in.urdf "${CMAKE_CURRENT_BINARY_DIR}/urdf/${NAME}.urdf")
  configure_file(yaml/${NAME}.in.yaml "${CMAKE_CURRENT_BINARY_DIR}/yaml/${NAME}.yaml")
  # Configure the alias accordingly
  configure_file(alias/${NAME}.in.yaml "${CMAKE_CURRENT_BINARY_DIR}/alias/${NAME}.yaml")

  # Install
  if(INSTALL_MUJOCO_OBJECTS)
    if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/xml/${NAME}.in.xml)
      configure_file(xml/${NAME}.in.xml "${CMAKE_CURRENT_BINARY_DIR}/xml/${NAME}.xml")
      configure_file(xml/${NAME}.in.yaml "${CMAKE_CURRENT_BINARY_DIR}/xml/${NAME}.yaml")

      install(FILES "${CMAKE_CURRENT_BINARY_DIR}/xml/${NAME}.xml" DESTINATION "${MC_MUJOCO_SHARE_DESTINATION}/object")
      install(FILES "${CMAKE_CURRENT_BINARY_DIR}/xml/${NAME}.yaml" DESTINATION "${MC_MUJOCO_SHARE_DESTINATION}")
    else()
      message(STATUS "MUJOCO model is not supported for ${NAME}")
    endif()
  endif()

  # Install the alias
  install(FILES "${CMAKE_CURRENT_BINARY_DIR}/alias/${NAME}.yaml" DESTINATION "${MC_ROBOTS_ALIASES_DIRECTORY}")
  install(FILES "${CMAKE_CURRENT_BINARY_DIR}/urdf/${NAME}.urdf" DESTINATION "${MODULE_INSTALL_LOCATION}/urdf")
  install(FILES "${CMAKE_CURRENT_BINARY_DIR}/yaml/${NAME}.yaml" DESTINATION "${MODULE_INSTALL_LOCATION}/yaml")
  install(DIRECTORY rsdf/${NAME} DESTINATION "${MODULE_INSTALL_LOCATION}/rsdf" FILES_MATCHING PATTERN "*.rsdf")
  install(DIRECTORY meshes/${NAME} DESTINATION "${MODULE_INSTALL_LOCATION}/meshes"
    FILES_MATCHING PATTERN "*.stl" PATTERN "*.STL" PATTERN "*.dae" PATTERN "*.DAE"
  )
endmacro()

add_yaml_module(box_lid)
add_yaml_module(box_no_lid)
add_yaml_module(shelf)
