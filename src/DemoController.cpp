#include "DemoController.h"

DemoController::DemoController
(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration &config)
    : BWC::BaselineWalkingController(rm, dt, config)
{
    for (const auto &k: config("robots").keys())
    {
        mc_rtc::log::info("DemoController config('robots'): {}", k);
        auto v = config("robots")(k);
        for (const auto &kk: v.keys())
        {
            mc_rtc::log::info("                               ('{}'): {}", k, kk);
        }
    }
    mc_rtc::log::success("DemoController init done ");
}

bool DemoController::run()
{
    return BWC::BaselineWalkingController::run();
}

void DemoController::reset(const mc_control::ControllerResetData &reset_data)
{
    BWC::BaselineWalkingController::reset(reset_data);
    
    m_box0PostureTask = std::make_shared<mc_tasks::PostureTask>
            (solver(), robot("zbox0").robotIndex(), 5, 100);
    solver().addTask(m_box0PostureTask);
    m_box1PostureTask = std::make_shared<mc_tasks::PostureTask>
            (solver(), robot("zbox1").robotIndex(), 5, 100);
    solver().addTask(m_box1PostureTask);

    m_box0KinematicConstraint = std::make_unique<mc_solver::KinematicsConstraint>
    (
        robots(),
        robot("zbox0").robotIndex(),
        solver().dt()
    );
    solver().addConstraintSet(m_box0KinematicConstraint);
    m_box1KinematicConstraint = std::make_unique<mc_solver::KinematicsConstraint>
    (
        robots(),
        robot("zbox1").robotIndex(),
        solver().dt()
    );
    solver().addConstraintSet(m_box1KinematicConstraint);
}
