#include "HoldBox.hpp"
#include <mc_control/Contact.h>
#include <mc_rtc/logging.h>
#include "../DemoController.h"
#include "./utils.h"

void HoldBox::configure(const mc_rtc::Configuration &config)
{
    mc_rtc::log::info("GoToDropoffPose:\n{}", config.dump(true, true));

    config("robotReferenceFrame", m_robotReferenceFrame);
    config("objectName", m_objectName);
    config("objectSurfaceLeftGripper", m_objectSurfaceLeftGripper);
    config("objectSurfaceRightGripper", m_objectSurfaceRightGripper);
    config("stiffness", m_stiffness);
    config("weight", m_weight);
    config("leftPositionRobot", m_leftPositionRobot);
    config("rightPositionRobot", m_rightPositionRobot);
    config("leftOrientationRobot", m_leftOrientationRobot);
    config("rightOrientationRobot", m_rightOrientationRobot);
}

void HoldBox::start(mc_control::fsm::Controller &ctl_)
{
    auto &ctl = static_cast<DemoController &>(ctl_);

    bool hasLeftContact = false, hasRightContact = false;

    for (const auto &c : ctl.contacts())
    {
        mc_rtc::log::info("contact: {}:{} <-> {}:{}", c.r1->c_str(), c.r1Surface, c.r2->c_str(), c.r2Surface);

        if (c.r1 == ctl.robot().name() && c.r1Surface == "LeftHandWrench" && c.r2 == m_objectName &&
            c.r2Surface == m_objectSurfaceLeftGripper)
            hasLeftContact = true;

        if (c.r1 == ctl.robot().name() && c.r1Surface == "RightHandWrench" && c.r2 == m_objectName &&
            c.r2Surface == m_objectSurfaceRightGripper)
            hasRightContact = true;
    }
    if (!hasLeftContact || !hasRightContact) mc_rtc::log::error("Didn't find box contacts");

    const double boxHalfWidth = 0.5 *
            (ctl.robot(m_objectName).frame(m_objectSurfaceLeftGripper).position().translation() -
             ctl.robot(m_objectName).frame(m_objectSurfaceRightGripper).position().translation())
                    .norm();

    m_leftPositionRobot.y()  = boxHalfWidth;
    m_rightPositionRobot.y() = -boxHalfWidth;

    m_leftGripperTask = std::make_shared<mc_tasks::RelativeEndEffectorTask>(
            "LeftHandWrench", ctl.robots(), 0, m_robotReferenceFrame, m_stiffness, m_weight);
    m_leftGripperTask->selectActiveJoints(ctl.solver(), LeftArmJoints);
    m_leftGripperTask->set_ef_pose({m_leftOrientationRobot, m_leftPositionRobot});
    ctl.solver().addTask(m_leftGripperTask);

    m_rightGripperTask = std::make_shared<mc_tasks::RelativeEndEffectorTask>(
            "RightHandWrench", ctl.robots(), 0, m_robotReferenceFrame, m_stiffness, m_weight);
    m_rightGripperTask->selectActiveJoints(ctl.solver(), RightArmJoints);
    m_rightGripperTask->set_ef_pose({m_rightOrientationRobot, m_rightPositionRobot});
    ctl.solver().addTask(m_rightGripperTask);
}

bool HoldBox::run(mc_control::fsm::Controller &ctl_)
{
    auto &ctl = static_cast<DemoController &>(ctl_);

    // This is a hack to ensure the object is visible in mc_mujoco because for some reason the
    // box position does not change in the visualization
    const auto setPosWCall = m_objectName + "::SetPosW";
    if (ctl.datastore().has(setPosWCall))
    {
        const auto &objectPosW = ctl.robot(m_objectName).posW();
        ctl.datastore().call<void, const sva::PTransformd &>(setPosWCall, objectPosW);
    }

    // is meant to run parallel to a walking task which will end
    return true;
}

void HoldBox::teardown(mc_control::fsm::Controller &ctl_)
{
    auto &ctl = static_cast<DemoController &>(ctl_);

    ctl.solver().removeTask(m_leftGripperTask);
    ctl.solver().removeTask(m_rightGripperTask);
}

EXPORT_SINGLE_STATE("HoldBox", HoldBox)
