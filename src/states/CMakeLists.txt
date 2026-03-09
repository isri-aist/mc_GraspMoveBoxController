add_fsm_state_simple(GoTo)

target_link_libraries(
        GoTo PUBLIC
        baseline_walking_controller::BaselineWalkingController
        baseline_footstep_planner::BaselineFootstepPlanner
        ${BWC_STATES_LIBRARIES}/FootstepPlannerState.so
)
set_target_properties(GoTo PROPERTIES
        BUILD_RPATH "${BWC_STATES_LIBRARIES}"
        INSTALL_RPATH "${BWC_STATES_LIBRARIES}"
)

add_fsm_state_simple(MoveHands)
add_fsm_state_simple(HoldBox)

add_fsm_state_simple(PickupBox)
add_fsm_state_simple(DropoffBox)

add_fsm_data_directory(data)
