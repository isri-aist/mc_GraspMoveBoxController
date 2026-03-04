#include "DemoController.h"

DemoController::DemoController(
    mc_rbdyn::RobotModulePtr      rm,
    double                        dt,
    const mc_rtc::Configuration & config
)
    : BWC::BaselineWalkingController(rm, dt, config)
{
    mc_rtc::log::success("DemoController init done ");
}

bool DemoController::run()
{
    return BWC::BaselineWalkingController::run();
}

void DemoController::reset(const mc_control::ControllerResetData & reset_data)
{
    BWC::BaselineWalkingController::reset(reset_data);
}