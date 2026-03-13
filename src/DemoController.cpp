#include "DemoController.h"

DemoController::DemoController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration &config)
    : BWC::BaselineWalkingController(rm, dt, config)
{
    config("CentroidalManager")("refComZ", m_refCoMZ);
    mc_rtc::log::success("DemoController init done ");
}

bool DemoController::run()
{
    return BWC::BaselineWalkingController::run();
}

void DemoController::reset(const mc_control::ControllerResetData &reset_data)
{
    BWC::BaselineWalkingController::reset(reset_data);
    gui()->addElement({"GMB"}, mc_rtc::gui::Label("Grasp Move Box Controller", [] { return ""; }));
}
