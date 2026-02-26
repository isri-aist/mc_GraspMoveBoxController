#include "DemoController_Initial.h"

#include "../DemoController.h"

void DemoController_Initial::configure(const mc_rtc::Configuration &config) {}

void DemoController_Initial::start(mc_control::fsm::Controller &ctl_)
{
    auto &ctl = static_cast<DemoController &>(ctl_);
}

bool DemoController_Initial::run(mc_control::fsm::Controller &ctl_)
{
    auto &ctl = static_cast<DemoController &>(ctl_);
    output("OK");
    return true;
}

void DemoController_Initial::teardown(mc_control::fsm::Controller &ctl_)
{
    auto &ctl = static_cast<DemoController &>(ctl_);
}

EXPORT_SINGLE_STATE("DemoController_Initial", DemoController_Initial)
