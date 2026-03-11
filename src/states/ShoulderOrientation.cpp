#include "ShoulderOrientation.hpp"
#include "../DemoController.h"
#include "./utils.h"

#include <BaselineWalkingController/CentroidalManager.h>
#include <SpaceVecAlg/PTransform.h>
#include <mc_rtc/gui/Label.h>
#include <mc_tasks/CoMTask.h>
#include <mc_tasks/TransformTask.h>

void ShoulderOrientation::configure(const mc_rtc::Configuration &config)
{
    mc_rtc::log::info("ShoulderOrientation:\n{}", config.dump(true, true));

    config("stiffness", m_stiffness);
    config("weight", m_weight);
    config("leftShoulderZAngle", m_leftShoulderZAngle);
    config("rightShoulderZAngle", m_rightShoulderZAngle);
}

void ShoulderOrientation::start(mc_control::fsm::Controller &ctl_)
{
    auto &ctl = static_cast<DemoController &>(ctl_);

    m_leftElbowOrientationTask = std::make_shared<mc_tasks::OrientationTask>(
            ctl.robot().frame(m_leftShoulderFrame), m_stiffness, m_weight / 2);
    m_leftElbowOrientationTask->selectActiveJoints(ctl.solver(), m_leftShoulderActiveJoints);
    m_leftElbowOrientationTask->orientation(sva::RotZ(m_leftShoulderZAngle));
    ctl.solver().addTask(m_leftElbowOrientationTask);

    m_rightElbowOrientationTask = std::make_shared<mc_tasks::OrientationTask>(
            ctl.robot().frame(m_rightShoulderFrame), m_stiffness, m_weight / 2);
    m_rightElbowOrientationTask->selectActiveJoints(ctl.solver(), m_rightShoulderActiveJoints);
    m_rightElbowOrientationTask->orientation(sva::RotZ(m_rightShoulderZAngle));
    ctl.solver().addTask(m_rightElbowOrientationTask);
}

bool ShoulderOrientation::run(mc_control::fsm::Controller &ctl_)
{
    // is meant to run parallel
    return true;
}

void ShoulderOrientation::teardown(mc_control::fsm::Controller &ctl_)
{
    auto &ctl = static_cast<DemoController &>(ctl_);

    ctl.solver().removeTask(m_leftElbowOrientationTask);
    ctl.solver().removeTask(m_rightElbowOrientationTask);
}

EXPORT_SINGLE_STATE("ShoulderOrientation", ShoulderOrientation)
