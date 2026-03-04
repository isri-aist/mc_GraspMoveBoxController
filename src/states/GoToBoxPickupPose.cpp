#include "GoToBoxPickupPose.h"

void GoToBoxPickupPose::configure(const mc_rtc::Configuration &config)
{
    mc_rtc::log::info("\n{}", config.dump(true, true));

    config("pickupPoseWorld", m_destinationPoseWorld);
}

void GoToBoxPickupPose::start(mc_control::fsm::Controller &ctl_)
{
    GoTo::start(ctl_);
}
bool GoToBoxPickupPose::run(mc_control::fsm::Controller &ctl_)
{
    return GoTo::run(ctl_);
}
void GoToBoxPickupPose::teardown(mc_control::fsm::Controller &ctl_)
{
    GoTo::teardown(ctl_);
}

EXPORT_SINGLE_STATE("GoToBoxPickupPose", GoToBoxPickupPose)
