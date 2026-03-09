#pragma once

#include <BaselineWalkingController/states/FootstepPlannerState.h>
#include <mc_control/fsm/State.h>

struct GoTo : BWC::FootstepPlannerState
{
        bool run(mc_control::fsm::Controller &ctl_) override;
        void start(mc_control::fsm::Controller &ctl_) override;
        void teardown(mc_control::fsm::Controller &ctl_) override;
        void configure(const mc_rtc::Configuration &config) override;

    protected:
        bool m_autoStart = false;
        bool m_planning  = false;
        bool m_started   = false;

        Eigen::Vector3d m_destinationPoseWorld;
};
