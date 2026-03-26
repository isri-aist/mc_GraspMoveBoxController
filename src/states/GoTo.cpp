#include "GoTo.hpp"
#include "../DemoController.h"

#include <BaselineWalkingController/FootManager.h>

void GoTo::configure(const mc_rtc::Configuration & config)
{
    m_config.load(config);

    FootstepPlannerState::configure(m_config);

    mc_rtc::log::info("GoTo:\n{}", m_config.dump(true, true));

    if (!m_config.has("destinationPoseWorld")) mc_rtc::log::error_and_throw("Configuration is missing fields");

    m_autoStart = m_config("autoStart", false);

    m_config("destinationPoseWorld", m_destinationPoseWorld);

    m_started = false;
}

void GoTo::start(mc_control::fsm::Controller & ctl_)
{
    FootstepPlannerState::start(ctl_);

    auto & ctl = static_cast<DemoController&>(ctl_);

    ctl.footManager_->clearFootstepQueue();

    auto start = [&ctl, this]
    {
        goalFootMidpose_ = {m_destinationPoseWorld.x(), m_destinationPoseWorld.y(), m_destinationPoseWorld.z()};
        triggered_       = true;
        m_planning       = true;
        mc_rtc::log::info("Footstep planner triggered");
    };

    if (m_autoStart) start();

    else
        ctl.gui()->addElement(
            {"GraspMoveBox"},
            mc_rtc::gui::Button(
                "Start",
                [start]
                {
                    start();
                }
                )
            );
}

bool GoTo::run(mc_control::fsm::Controller & ctl_)
{
    auto & ctl = static_cast<DemoController&>(ctl_);

    if (m_planning && footstepPlanner_->solution_.is_solved)
    {
        mc_rtc::log::info("Footstep planner found solution");
        m_started  = true;
        m_planning = false;
    }

    if (m_started && ctl.footManager_->footstepQueue().empty())
    {
        output("OK");
        return true;
    }

    return false;
}

void GoTo::teardown(mc_control::fsm::Controller & ctl_)
{
    FootstepPlannerState::teardown(ctl_);
}

EXPORT_SINGLE_STATE("GoTo", GoTo)
