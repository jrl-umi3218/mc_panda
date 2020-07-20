add_fsm_data_directory(data)

add_fsm_state_simple(WaitForCollisionState)
target_link_libraries(WaitForCollisionState PUBLIC mc_panda::devices)

#set(states_HDR
#  src/states/WaitForCollisionState.h
#)
#set(states_SRC
#  src/states/WaitForCollisionState.cpp
#)
#
#add_library(mc_panda_states SHARED ${states_SRC} ${states_HDR})
#add_library(mc_panda::states ALIAS mc_panda_states)
#set_target_properties(mc_panda_states PROPERTIES EXPORT_NAME states)
#target_include_directories(mc_panda_states PUBLIC
#  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
#  $<INSTALL_INTERFACE:include>)
#target_link_libraries(mc_panda_states PUBLIC mc_rtc::mc_rbdyn mc_panda::devices)
#install(FILES ${states_HDR} DESTINATION include/mc_panda/states)
#install(TARGETS mc_panda_states
#  EXPORT mc_pandaTargets
#  ARCHIVE DESTINATION lib
#  LIBRARY DESTINATION lib
#  RUNTIME DESTINATION bin)

