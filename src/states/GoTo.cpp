#include "GoTo.h"
#include "../DemoController.h"

#include <BaselineWalkingController/FootManager.h>

void GoTo::configure(const mc_rtc::Configuration &config)
{
    FootstepPlannerState::configure(config);

    mc_rtc::log::info("GoTo:\n{}", config.dump(true, true));
    config("autoStart", m_autoStart);
    config("destinationPoseWorld", m_destinationPoseWorld);
}

void GoTo::start(mc_control::fsm::Controller &ctl_)
{
    FootstepPlannerState::start(ctl_);

    auto &ctl = static_cast<DemoController &>(ctl_);

    ctl.footManager_->clearFootstepQueue();

    auto start = [&ctl, this]
    {
        goalFootMidpose_ = {m_destinationPoseWorld.x(), m_destinationPoseWorld.y(), m_destinationPoseWorld.z()};
        triggered_       = true;
        m_planning       = true;
    };

    if (m_autoStart)
        start();

    else
        ctl.gui()->addElement({"GraspMoveBox"}, mc_rtc::gui::Button("Start", [start] { start(); }));
}

bool GoTo::run(mc_control::fsm::Controller &ctl_)
{
    auto &ctl = static_cast<DemoController &>(ctl_);

    if (m_planning && footstepPlanner_->solution_.is_solved)
    {
        mc_rtc::log::info("Footstep planner found solution");
        m_started = true;
        m_planning = false;
    }
    if (m_planning) return false;
    if (m_started && !ctl.footManager_->footstepQueue().empty()) return false;

    output("OK");
    return true;
}

void GoTo::teardown(mc_control::fsm::Controller &ctl_)
{
    FootstepPlannerState::teardown(ctl_);
}

EXPORT_SINGLE_STATE("GoTo", GoTo)
