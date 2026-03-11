#include "PickupBox.hpp"
#include "../DemoController.h"
#include "./utils.h"

#include <BaselineWalkingController/CentroidalManager.h>
#include <SpaceVecAlg/PTransform.h>
#include <mc_rtc/gui/Label.h>
#include <mc_tasks/CoMTask.h>
#include <mc_tasks/TransformTask.h>

void PickupBox::configure(const mc_rtc::Configuration &config)
{
    mc_rtc::log::info("PickupBox:\n{}", config.dump(true, true));

    config("robotReferenceFrame", m_robotReferenceFrame);
    config("objectName", m_objectName);
    config("objectSurfaceLeftGripper", m_objectSurfaceLeftGripper);
    config("objectSurfaceRightGripper", m_objectSurfaceRightGripper);
    config("stiffness", m_stiffness);
    config("weight", m_weight);
    config("timeout", m_timeout);
    config("completionEval", m_completionEval);
    config("completionSpeed", m_completionSpeed);
    config("crouchOffset", m_crouchOffset);
    config("removeContactsAtTeardown", m_removeContactAtTeardown);
    config("manualPhaseChange", m_manualPhaseChange);
    config("leftGripperContactOffset", m_leftGripperContactOffset);
    config("rightGripperContactOffset", m_rightGripperContactOffset);
    config("approachOffset", m_approachOffset);
    config("leftCarryPositionRobot", m_leftCarryPositionRobot);
    config("rightCarryPositionRobot", m_rightCarryPositionRobot);
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

    m_contactAdded     = false;
    m_allowPhaseChange = !m_manualPhaseChange;
}

void PickupBox::start(mc_control::fsm::Controller &ctl_)
{
    auto &ctl = static_cast<DemoController &>(ctl_);

    m_leftGripperTask = std::make_shared<mc_tasks::TransformTask>(
            m_gripperSurfaceLeftGripper, ctl.robots(), 0, m_stiffness, m_weight);
    m_leftGripperTask->selectActiveJoints(ctl.solver(), LeftArmJoints);
    m_leftGripperTask->target(ctl.robot().frame(m_gripperSurfaceLeftGripper).position());
    ctl.solver().addTask(m_leftGripperTask);

    m_rightGripperTask = std::make_shared<mc_tasks::TransformTask>(
            m_gripperSurfaceRightGripper, ctl.robots(), 0, m_stiffness, m_weight);
    m_rightGripperTask->selectActiveJoints(ctl.solver(), RightArmJoints);
    m_rightGripperTask->target(ctl.robot().frame(m_gripperSurfaceRightGripper).position());
    ctl.solver().addTask(m_rightGripperTask);

    m_leftContact = mc_control::Contact(
            ctl.robot().name(),
            ctl.robot(m_objectName).name(),
            "LeftHandWrench",
            m_objectSurfaceLeftGripper,
            mc_rbdyn::Contact::defaultFriction,
            Eigen::Vector6d::Ones());

    m_rightContact = mc_control::Contact(
            ctl.robot().name(),
            ctl.robot(m_objectName).name(),
            "RightHandWrench",
            m_objectSurfaceRightGripper,
            mc_rbdyn::Contact::defaultFriction,
            Eigen::Vector6d::Ones());

    m_boxHalfWidth = 0.5 *
            (ctl.robot(m_objectName).frame(m_objectSurfaceLeftGripper).position().translation() -
             ctl.robot(m_objectName).frame(m_objectSurfaceRightGripper).position().translation())
                    .norm();

    ctl.gui()->addElement(
            {"GraspMoveBox"},
            mc_rtc::gui::Button(
                    "Force Add Contacts",
                    [&ctl, this]
                    {
                        ctl.addContact(m_leftContact);
                        ctl.addContact(m_rightContact);
                    }));

    m_leftCarryPositionRobot.y()  = m_boxHalfWidth;
    m_rightCarryPositionRobot.y() = -m_boxHalfWidth;
    m_refComZ                     = ctl.comTask_->com().z();

    ctl.gui()->addElement({"GraspMoveBox"}, mc_rtc::gui::Button("Next Phase", [this] { m_allowPhaseChange = true; }));

    ctl.gui()->addElement(
            {"GraspMoveBox"},
            mc_rtc::gui::Label(
                    "Left gripper distance to box and speed",
                    [this]
                    {
                        std::string data = std::to_string(m_leftGripperTask->eval().norm());
                        data += "\t";
                        data += std::to_string(m_leftGripperTask->speed().norm());
                        return data;
                    }));
    ctl.gui()->addElement(
            {"GraspMoveBox"},
            mc_rtc::gui::Label(
                    "Right gripper distance to box and speed",
                    [this]
                    {
                        std::string data = std::to_string(m_rightGripperTask->eval().norm());
                        data += "\t";
                        data += std::to_string(m_rightGripperTask->speed().norm());
                        return data;
                    }));
}

bool PickupBox::run(mc_control::fsm::Controller &ctl_)
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
        if (!m_allowPhaseChange) return false;
        if (m_manualPhaseChange) m_allowPhaseChange = false;

        mc_rtc::log::info("Now in approach phase");
        m_phase = Phase::ApproachBox;

        // set to max double to deactivate the timeout
        m_startTime = std::numeric_limits<double>::max();

        m_leftGripperTask->targetSurface(
                ctl.robot(m_objectName).robotIndex(),
                m_objectSurfaceLeftGripper,
                {m_leftOrientationBox, (m_leftApproachOffsetBox + m_leftGraspOffsetBox).eval()});

        m_rightGripperTask->targetSurface(
                ctl.robot(m_objectName).robotIndex(),
                m_objectSurfaceRightGripper,
                {m_rightOrientationBox, (m_rightApproachOffsetBox + m_rightGraspOffsetBox).eval()});

        ctl.centroidalManager_->setRefComZ(m_refComZ - m_crouchOffset, ctl.t(), m_crouchOffset * 20.0);

        return false;
    }

    bool completed =
            (m_leftGripperTask->eval().norm() < m_completionEval &&
             m_leftGripperTask->speed().norm() < m_completionSpeed &&
             m_rightGripperTask->eval().norm() < m_completionEval &&
             m_rightGripperTask->speed().norm() < m_completionSpeed);


    if (m_phase == Phase::ApproachBox && completed)
    {
        if (!m_allowPhaseChange) return false;
        if (m_manualPhaseChange) m_allowPhaseChange = false;

        mc_rtc::log::info("Now in grasping phase");
        m_phase = Phase::GraspBox;

        m_leftGripperTask->targetSurface(
                ctl.robot(m_objectName).robotIndex(),
                m_objectSurfaceLeftGripper,
                {m_leftOrientationBox, m_leftGraspOffsetBox});

        m_rightGripperTask->targetSurface(
                ctl.robot(m_objectName).robotIndex(),
                m_objectSurfaceRightGripper,
                {m_rightOrientationBox, m_rightGraspOffsetBox});

        return false;
    }

    if (m_phase == Phase::GraspBox && completed)
    {
        if (!m_allowPhaseChange) return false;
        if (m_manualPhaseChange) m_allowPhaseChange = false;

        mc_rtc::log::info("Now in lift phase");
        m_phase = Phase::RaiseBox;

        ctl.addContact(m_leftContact);
        ctl.addContact(m_rightContact);
        m_contactAdded = true;

        m_leftGripperTask->target(
                ctl.robot().frame(m_robotReferenceFrame),
                {m_leftOrientationRobot, m_leftCarryPositionRobot + m_leftGraspOffsetRobot});

        m_rightGripperTask->target(
                ctl.robot().frame(m_robotReferenceFrame),
                {m_rightOrientationRobot, m_rightCarryPositionRobot + m_rightGraspOffsetRobot});

        ctl.centroidalManager_->setRefComZ(m_refComZ, ctl.t(), m_crouchOffset * 20.0);

        return false;
    }

    if (m_phase == Phase::RaiseBox && completed)
    {
        output("OK");
        return true;
    }

    return false;
}

void PickupBox::teardown(mc_control::fsm::Controller &ctl_)
{
    auto &ctl = static_cast<DemoController &>(ctl_);

    ctl.solver().removeTask(m_leftGripperTask);
    ctl.solver().removeTask(m_rightGripperTask);

    if (m_contactAdded && m_removeContactAtTeardown)
    {
        ctl.removeContact(m_leftContact);
        ctl.removeContact(m_rightContact);
        m_contactAdded = false;
    }

    ctl.gui()->removeElement({"GraspMoveBox"}, "Next Phase");
    ctl.gui()->removeElement({"GraspMoveBox"}, "Left gripper distance to box and speed");
    ctl.gui()->removeElement({"GraspMoveBox"}, "Right gripper distance to box and speed");
    ctl.gui()->removeElement({"GraspMoveBox"}, "Force Add Contacts");
}

EXPORT_SINGLE_STATE("PickupBox", PickupBox)
