#pragma once

#include <mc_control/mc_controller.h>
#include <mc_control/fsm/Controller.h>
#include <BaselineWalkingController/BaselineWalkingController.h>
#include "api.h"

struct DemoController_DLLAPI DemoController : public BWC::BaselineWalkingController
{
    DemoController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration &config);

    bool run() override;

    void reset(const mc_control::ControllerResetData &reset_data) override;

private:
    std::shared_ptr<mc_tasks::PostureTask>
            m_box0PostureTask,
            m_box1PostureTask;

    std::unique_ptr<mc_solver::KinematicsConstraint>
            m_box0KinematicConstraint,
            m_box1KinematicConstraint;
};
