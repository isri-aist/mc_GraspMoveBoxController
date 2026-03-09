add_library(GoTo SHARED GoTo.cpp)
message(STATUS "${baseline_walking_controller_LIBRARY}")
target_link_libraries(
        GoTo
        mc_rtc::mc_control
        baseline_walking_controller::BaselineWalkingController
        baseline_footstep_planner::BaselineFootstepPlanner
        ${BWC_STATES_LIBRARIES}/FootstepPlannerState.so
)
set_target_properties(GoTo PROPERTIES
        BUILD_RPATH "${BWC_STATES_LIBRARIES}"
        INSTALL_RPATH "${BWC_STATES_LIBRARIES}"
)
install(TARGETS GoTo
        EXPORT ${PROJECT_NAME}Targets
        LIBRARY DESTINATION "${MC_STATES_RUNTIME_INSTALL_PREFIX}"
)

add_fsm_state_simple(MoveHands)

add_fsm_state_simple(GoToBoxPickupPose)
target_link_libraries(GoToBoxPickupPose PUBLIC GoTo)

add_fsm_state_simple(GoToBoxDropoffPose)
target_link_libraries(GoToBoxDropoffPose PUBLIC GoTo)

add_fsm_state_simple(PickupBox)
add_fsm_state_simple(DropoffBox)

add_fsm_data_directory(data)
