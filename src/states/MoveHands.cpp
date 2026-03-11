#include "MoveHands.hpp"
#include "../DemoController.h"
#include "utils.h"

void MoveHands::configure(const mc_rtc::Configuration &config)
{
    mc_rtc::log::info("MoveHands:\n{}", config.dump(true, true));

    // if (!(config.has("leftHandTargetPositionRobot") && config.has("rightHandTargetPositionRobot")))
    //     mc_rtc::log::error_and_throw("Missing required config fields");

    config("leftHandTargetPositionRobot", m_leftHandTargetPositionRobot);
    config("rightHandTargetPositionRobot", m_rightHandTargetPositionRobot);

    config("stiffness", m_stiffness);
    config("weight", m_weight);
    config("leftHandFrame", m_leftHandFrame);
    config("rightHandFrame", m_rightHandFrame);
}

void MoveHands::start(mc_control::fsm::Controller &ctl_)
{
    auto &ctl = static_cast<DemoController &>(ctl_);

    // for (auto &j : ctl.robot().mb().joints()) mc_rtc::log::info("{}", j.name());

    m_leftGripperTask = std::make_shared<mc_tasks::RelativeEndEffectorTask>(
            m_leftHandFrame, ctl.robots(), 0, m_robotReferenceFrame, m_stiffness, m_weight);
    m_leftGripperTask->selectActiveJoints(ctl.solver(), LeftArmJoints);
    m_leftGripperTask->set_ef_pose({m_leftHandTargetOrientationRobot, m_leftHandTargetPositionRobot});
    ctl.solver().addTask(m_leftGripperTask);

    m_rightGripperTask = std::make_shared<mc_tasks::RelativeEndEffectorTask>(
            "RightHandWrench", ctl.robots(), 0, m_robotReferenceFrame, m_stiffness, m_weight);
    m_rightGripperTask->selectActiveJoints(ctl.solver(), RightArmJoints);
    m_rightGripperTask->set_ef_pose({m_rightHandTargetOrientationRobot, m_rightHandTargetPositionRobot});
    ctl.solver().addTask(m_rightGripperTask);
}

bool MoveHands::run(mc_control::fsm::Controller &ctl_)
{
    if (!m_waitForEvalThreshold) return true;
    return false;
}

void MoveHands::teardown(mc_control::fsm::Controller &ctl_)
{
    auto &ctl = static_cast<DemoController &>(ctl_);

    ctl.solver().removeTask(m_leftGripperTask);
    ctl.solver().removeTask(m_rightGripperTask);
}

EXPORT_SINGLE_STATE("MoveHands", MoveHands)
