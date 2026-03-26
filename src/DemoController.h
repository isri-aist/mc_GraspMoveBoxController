#pragma once

#include <BaselineWalkingController/BaselineWalkingController.h>
#include <mc_control/fsm/Controller.h>
#include "api.h"

struct DemoController_DLLAPI DemoController : public BWC::BaselineWalkingController
{
    DemoController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

    bool run() override;

    void reset(const mc_control::ControllerResetData & reset_data) override;

    void setRobotPoseFromGui(const sva::PTransformd & tf);

    void reinitializeAfterTeleport();

    double m_refCoMZ = 0.94;
};