add_fsm_data_directory(data)

add_fsm_state_simple(WaitForCollisionState)
add_fsm_state_simple(PandaStop)
target_link_libraries(WaitForCollisionState PUBLIC mc_panda::devices)
target_link_libraries(PandaStop PUBLIC mc_panda::devices)
