add_fsm_state(GoTo GoTo.cpp GoTo.hpp)

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

add_fsm_state(MoveHands MoveHands.cpp MoveHands.hpp)
add_fsm_state(HoldBox HoldBox.cpp HoldBox.hpp)
add_fsm_state(ShoulderOrientation ShoulderOrientation.cpp ShoulderOrientation.hpp)

add_fsm_state(PickupBox PickupBox.cpp PickupBox.hpp)
add_fsm_state(DropoffBox DropoffBox.cpp DropoffBox.hpp)

add_fsm_data_directory(data)
