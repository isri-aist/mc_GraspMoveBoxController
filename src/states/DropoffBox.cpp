#include "DropoffBox.hpp"

#include <Eigen/Geometry>
#include <mc_rtc/gui/Label.h>
#include <mc_tasks/CoMTask.h>

#include "../DemoController.h"
#include "BaselineWalkingController/CentroidalManager.h"
#include "BaselineWalkingController/FootManager.h"
#include "utils.h"

void DropoffBox::configure(const mc_rtc::Configuration &config)
{
    mc_rtc::log::info("DropoffBox:\n{}", config.dump(true, true));

    config("robotReferenceFrame", m_robotReferenceFrame);
    config("objectName", m_objectName);
    config("objectSurfaceLeftGripper", m_objectSurfaceLeftGripper);
    config("objectSurfaceRightGripper", m_objectSurfaceRightGripper);
    config("stiffness", m_stiffness);
    config("weight", m_weight);
    config("timeout", m_timeout);
    config("completionEval", m_completionEval);
    config("completionSpeed", m_completionSpeed);
    config("leftShoulderZAngle", m_leftShoulderZAngle);
    config("rightShoulderZAngle", m_rightShoulderZAngle);
    config("crouchOffset", m_crouchOffset);
    config("removeContactsAtTeardown", m_removeContactAtTeardown);
    config("manualPhaseChange", m_manualPhaseChange);
    config("leftGripperContactOffset", m_leftGripperContactOffset);
    config("rightGripperContactOffset", m_rightGripperContactOffset);
    config("approachOffset", m_approachOffset);
    config("leftDropPositionRobot", m_leftDropPositionRobot);
    config("rightDropPositionRobot", m_rightDropPositionRobot);
    config("leftOrientationBox", m_leftOrientationBox);
    config("rightOrientationBox", m_rightOrientationBox);
    config("leftRaisePositionRobot", m_leftRaisePositionRobot);
    config("rightRaisePositionRobot", m_rightRaisePositionRobot);
    config("leftOrientationRobot", m_leftOrientationRobot);
    config("rightOrientationRobot", m_rightOrientationRobot);

    m_leftGraspOffsetRobot  = {0.0, m_leftGripperContactOffset, 0.0};
    m_leftGraspOffsetBox    = {0.0, 0.0, m_leftGripperContactOffset};
    m_rightGraspOffsetRobot = {0.0, -m_rightGripperContactOffset, 0.0};
    m_rightGraspOffsetBox   = {0.0, 0.0, m_rightGripperContactOffset};

    m_leftApproachOffsetBox  = {0.0, 0.0, m_approachOffset};
    m_rightApproachOffsetBox = {0.0, 0.0, m_approachOffset};

    m_allowPhaseChange = !m_manualPhaseChange;
}

void DropoffBox::start(mc_control::fsm::Controller &ctl_)
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
    if (!hasLeftContact || !hasRightContact) mc_rtc::log::error_and_throw("Didn't find box contacts");
    m_contactAdded = true;

    m_leftGripperTask =
            std::make_shared<mc_tasks::TransformTask>("LeftHandSupportPlate", ctl.robots(), 0, m_stiffness, m_weight);
    m_leftGripperTask->selectActiveJoints(ctl.solver(), LeftArmJoints);

    m_rightGripperTask =
            std::make_shared<mc_tasks::TransformTask>("RightHandSupportPlate", ctl.robots(), 0, m_stiffness, m_weight);
    m_rightGripperTask->selectActiveJoints(ctl.solver(), RightArmJoints);

    // todo: *ArmJoints, quaternion orientation and orientation active joint must be loaded as config
    m_leftElbowOrientationTask = std::make_shared<mc_tasks::OrientationTask>(
            ctl.robot().frame("L_SHOULDER_Y_LINK"), m_stiffness, m_weight / 2);
    m_leftElbowOrientationTask->selectActiveJoints(ctl.solver(), {"L_SHOULDER_Y"});
    m_leftElbowOrientationTask->orientation(
            ctl.robot().frame(m_robotReferenceFrame).position().rotation() * sva::RotZ(m_leftShoulderZAngle));

    ctl.solver().addTask(m_leftElbowOrientationTask);

    m_rightElbowOrientationTask = std::make_shared<mc_tasks::OrientationTask>(
            ctl.robot().frame("R_SHOULDER_Y_LINK"), m_stiffness, m_weight / 2);
    m_rightElbowOrientationTask->selectActiveJoints(ctl.solver(), {"R_SHOULDER_Y"});
    m_rightElbowOrientationTask->orientation(
            ctl.robot().frame(m_robotReferenceFrame).position().rotation() * sva::RotZ(m_rightShoulderZAngle));

    ctl.solver().addTask(m_rightElbowOrientationTask);

    m_boxHalfWidth = 0.5 *
            (ctl.robot(m_objectName).frame(m_objectSurfaceLeftGripper).position().translation() -
             ctl.robot(m_objectName).frame(m_objectSurfaceRightGripper).position().translation())
                    .norm();

    m_leftDropPositionRobot.y()  = m_boxHalfWidth;
    m_rightDropPositionRobot.y() = -m_boxHalfWidth;
    m_refComZ                    = ctl.comTask_->com().z();
}

bool DropoffBox::run(mc_control::fsm::Controller &ctl_)
{
    auto &ctl = static_cast<DemoController &>(ctl_);

    // This is a hack to ensure the object is visible in mc_mujoco because for some reason the
    // box position does not change in the visualization
    if (m_contactAdded)
    {
        const auto setPosWCall = m_objectName + "::SetPosW";
        if (ctl.datastore().has(setPosWCall))
        {
            const auto &objectPosW = ctl.robot(m_objectName).posW();
            ctl.datastore().call<void, const sva::PTransformd &>(setPosWCall, objectPosW);
        }
    }

    if (m_phase == Phase::None)
    {
        mc_rtc::log::info("Now in lower box phase");
        m_phase = Phase::LowerBox;

        m_leftGripperTask->target(
                ctl.robot().frame(m_robotReferenceFrame),
                {m_leftOrientationRobot, m_leftDropPositionRobot + m_leftGraspOffsetRobot});
        ctl.solver().addTask(m_leftGripperTask);

        m_rightGripperTask->target(
                ctl.robot().frame(m_robotReferenceFrame),
                {m_rightOrientationRobot, m_rightDropPositionRobot + m_rightGraspOffsetRobot});
        ctl.solver().addTask(m_rightGripperTask);

        ctl.centroidalManager_->setRefComZ(m_refComZ - m_crouchOffset, ctl.t(), 1.0);
    }

    bool completed =
            (m_leftGripperTask->eval().norm() < m_completionEval &&
             m_leftGripperTask->speed().norm() < m_completionSpeed &&
             m_rightGripperTask->eval().norm() < m_completionEval &&
             m_rightGripperTask->speed().norm() < m_completionSpeed);

    if (m_phase == Phase::RemoveHands && m_startTime + m_timeout < ctl.t())
    {
        mc_rtc::log::info("remove hands timed out");
        completed = true;
    }

    if (m_phase == Phase::LowerBox && completed)
    {
        if (!m_allowPhaseChange) return false;
        if (m_manualPhaseChange) m_allowPhaseChange = false;

        mc_rtc::log::info("Now in drop box phase");
        m_phase = Phase::DropBox;

        if (m_contactAdded)
        {
            ctl.clearContacts();
            m_contactAdded = false;
        }

        m_leftGripperTask->targetSurface(
                ctl.robot(m_objectName).robotIndex(),
                m_objectSurfaceLeftGripper,
                {m_leftOrientationBox, (m_leftApproachOffsetBox + m_leftGraspOffsetBox).eval()});

        m_rightGripperTask->targetSurface(
                ctl.robot(m_objectName).robotIndex(),
                m_objectSurfaceRightGripper,
                {m_rightOrientationBox, (m_rightApproachOffsetBox + m_rightGraspOffsetBox).eval()});
    }

    if (m_phase == Phase::DropBox && completed)
    {
        if (!m_allowPhaseChange) return false;
        if (m_manualPhaseChange) m_allowPhaseChange = false;

        mc_rtc::log::info("Now in remove hands phase");
        m_phase = Phase::RemoveHands;

        m_startTime = ctl.t();

        ctl.solver().removeTask(m_leftElbowOrientationTask);
        ctl.solver().removeTask(m_rightElbowOrientationTask);

        m_leftGripperTask->target(
                ctl.robot().frame(m_robotReferenceFrame), {m_leftOrientationRobot, m_leftRaisePositionRobot});

        m_rightGripperTask->target(
                ctl.robot().frame(m_robotReferenceFrame), {m_rightOrientationRobot, m_rightRaisePositionRobot});

        ctl.centroidalManager_->setRefComZ(m_refComZ, ctl.t(), 1.0);

        return false;
    }

    if (m_phase == Phase::RemoveHands && completed)
    {
        output("OK");
        return true;
    }

    return false;
}

void DropoffBox::teardown(mc_control::fsm::Controller &ctl_)
{
    auto &ctl = static_cast<DemoController &>(ctl_);

    ctl.solver().removeTask(m_leftGripperTask);
    ctl.solver().removeTask(m_rightGripperTask);

    if (m_contactAdded && m_removeContactAtTeardown)
    {
        ctl.clearContacts();
        m_contactAdded = false;
    }
}

EXPORT_SINGLE_STATE("DropoffBox", DropoffBox)
