add_library(GoTo SHARED GoTo.cpp)
target_link_libraries(GoTo mc_rtc::mc_control)
install(TARGETS GoTo
        EXPORT ${PROJECT_NAME}Targets
        LIBRARY DESTINATION "${MC_STATES_RUNTIME_INSTALL_PREFIX}"
)

add_fsm_state_simple(GoToBoxPickupPose)
target_link_libraries(GoToBoxPickupPose PUBLIC GoTo)

add_fsm_state_simple(GoToBoxDropoffPose)
target_link_libraries(GoToBoxDropoffPose PUBLIC GoTo)

add_fsm_state_simple(PickupBox)

add_fsm_data_directory(data)
