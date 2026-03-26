#include "DemoController.h"

#include <BaselineWalkingController/CentroidalManager.h>
#include <BaselineWalkingController/FootManager.h>

#include <mc_tasks/CoMTask.h>
#include <mc_tasks/FirstOrderImpedanceTask.h>
#include <mc_tasks/OrientationTask.h>

DemoController::DemoController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
    : BWC::BaselineWalkingController(rm, dt, config)
{
    config("CentroidalManager")("refComZ", m_refCoMZ);
    mc_rtc::log::success("DemoController init done ");
}

bool DemoController::run()
{
    return BWC::BaselineWalkingController::run();
}

void DemoController::reset(const mc_control::ControllerResetData & reset_data)
{
    BWC::BaselineWalkingController::reset(reset_data);
    gui()->addElement(
        {"GMB"},
        mc_rtc::gui::Label(
            "Grasp Move Box Controller",
            []
            {
                return "";
            }
            )
        );
    gui()->addElement(
        {"GMB"},
        mc_rtc::gui::Transform(
            "Robot",
            [this]
            {
                return robot().posW();
            },
            [this](const sva::PTransformd & tf)
            {
                setRobotPoseFromGui(tf);
            }
            )
        );
}

void applyPose(mc_rbdyn::Robot & r, const sva::PTransformd & tf)
{
    r.posW(tf);
    r.velW(sva::MotionVecd::Zero());
    r.accW(sva::MotionVecd::Zero());
}

void DemoController::setRobotPoseFromGui(const sva::PTransformd & tf)
{
    applyPose(robot(), tf);
    applyPose(outputRobot(), tf);

    const auto setPosWCall = robot().name() + "::SetPosW";
    if (datastore().has(setPosWCall))
    {
        // In simulation, let the backend own realRobot state and synchronize it.
        datastore().call<void, const sva::PTransformd&>(setPosWCall, tf);
    }
    else
    {
        // Fallback path when no external simulator bridge is available.
        applyPose(realRobot(), tf);
        applyPose(outputRealRobot(), tf);
    }

    // Teleporting the base introduces a discontinuity; reset observers to
    // reinitialize anchor-frame history and velocity filters from the new pose.
    resetObserverPipelines();

    // Rebuild walking references from the new world pose.
    reinitializeAfterTeleport();
}

void DemoController::reinitializeAfterTeleport()
{
    if (comTask_)
    {
        comTask_->reset();
    }
    if (baseOriTask_)
    {
        baseOriTask_->reset();
    }
    for (auto & [foot, task] : footTasks_)
    {
        static_cast<void>(foot);
        if (task)
        {
            task->reset();
        }
    }

    if (footManager_)
    {
        footManager_->clearFootstepQueue();
        footManager_->reset();
    }
    if (centroidalManager_)
    {
        centroidalManager_->reset();
        centroidalManager_->setAnchorFrame();
    }
    enableManagerUpdate_ = true;
}