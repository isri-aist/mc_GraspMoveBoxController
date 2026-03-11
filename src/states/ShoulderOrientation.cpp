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
    config("leftShoulderAngle", m_leftShoulderAngle);
    config("rightShoulderAngle", m_rightShoulderAngle);
}

void ShoulderOrientation::start(mc_control::fsm::Controller &ctl_)
{
    auto &ctl = static_cast<DemoController &>(ctl_);

    // for (auto &j : ctl.robot().mb().joints()) mc_rtc::log::info("{}", j.name());

    m_leftElbowOrientationTask =
            std::make_shared<mc_tasks::OrientationTask>(ctl.robot().frame(m_leftShoulderFrame), m_stiffness, m_weight);
    ctl.solver().addTask(m_leftElbowOrientationTask);

    m_rightElbowOrientationTask =
            std::make_shared<mc_tasks::OrientationTask>(ctl.robot().frame(m_rightShoulderFrame), m_stiffness, m_weight);
    ctl.solver().addTask(m_rightElbowOrientationTask);

    setOrientationTaskGoal(ctl_);
}

bool ShoulderOrientation::run(mc_control::fsm::Controller &ctl_)
{
    setOrientationTaskGoal(ctl_);

    // is meant to run parallel
    return true;
}

void ShoulderOrientation::teardown(mc_control::fsm::Controller &ctl_)
{
    auto &ctl = static_cast<DemoController &>(ctl_);

    ctl.solver().removeTask(m_leftElbowOrientationTask);
    ctl.solver().removeTask(m_rightElbowOrientationTask);
}

void ShoulderOrientation::setOrientationTaskGoal(mc_control::fsm::Controller &ctl_) const
{
    auto &ctl = static_cast<DemoController &>(ctl_);

    auto &leftReferenceFrame  = ctl.robot().frame(m_leftReferenceFrame).position().rotation();
    auto &rightReferenceFrame = ctl.robot().frame(m_rightReferenceFrame).position().rotation();

    const Eigen::Matrix3d leftRotation  = sva::RotX(m_leftShoulderAngle);
    const Eigen::Matrix3d rightRotation = sva::RotX(m_rightShoulderAngle);

    const Eigen::Matrix3d leftFinal = leftRotation * leftReferenceFrame;
    const Eigen::Matrix3d rightFinal = rightRotation * rightReferenceFrame;

    m_leftElbowOrientationTask->orientation(leftRotation * leftReferenceFrame);
    m_rightElbowOrientationTask->orientation(rightRotation * rightReferenceFrame);
}

EXPORT_SINGLE_STATE("ShoulderOrientation", ShoulderOrientation)
