#include "PickupBox.h"
#include "../DemoController.h"
#include "./utils.h"

#include <mc_rtc/gui/Label.h>
#include <mc_tasks/TransformTask.h>


void PickupBox::configure(const mc_rtc::Configuration &config)
{
    mc_rtc::log::info("\n{}", config.dump(true, true));

    config("robotReferenceFrame", m_robotReferenceFrame);
    config("objectName", m_objectName);
    config("objectSurfaceLeftGripper", m_objectSurfaceLeftGripper);
    config("objectSurfaceRightGripper", m_objectSurfaceRightGripper);
    config("stiffness", m_stiffness);
    config("weight", m_weight);
    config("Timeout", m_Timeout);
    config("completionEval", m_completionEval);
    config("completionSpeed", m_completionSpeed);
    config("removeContactsAtTeardown", m_removeContactAtTeardown);
    config("manualPhaseChange", m_manualPhaseChange);
    config("leftApproachOffset", m_leftApproachOffset);
    config("rightApproachOffset", m_rightApproachOffset);
    config("leftGraspOffset", m_leftGraspOffset);
    config("rightGraspOffset", m_rightGraspOffset);
    config("leftCarryPositionRobot", m_leftCarryPositionRobot);
    config("rightCarryPositionRobot", m_rightCarryPositionRobot);
    config("leftCarryOrientationRobot", m_leftCarryOrientationRobot);
    config("rightCarryOrientationRobot", m_rightCarryOrientationRobot);
    config("leftRaisePositionRobot", m_leftRaisePositionRobot);
    config("rightRaisePositionRobot", m_rightRaisePositionRobot);
    config("leftRaiseOrientationRobot", m_leftRaiseOrientationRobot);
    config("rightRaiseOrientationRobot", m_rightRaiseOrientationRobot);

    m_contactAdded     = false;
    m_allowPhaseChange = !m_manualPhaseChange;
}

void PickupBox::start(mc_control::fsm::Controller &ctl_)
{
    auto &ctl = static_cast<DemoController &>(ctl_);

    m_leftGripperTask =
            std::make_shared<mc_tasks::TransformTask>("LeftHandSupportPlate", ctl.robots(), 0, m_stiffness, m_weight);
    m_leftGripperTask->selectActiveJoints(ctl.solver(), LeftArmJoints);

    // todo: *ArmJoints, quaternion orientation and orientation active joint must be loaded as config
    m_leftElbowOrientationTask = std::make_shared<mc_tasks::OrientationTask>(
            ctl.robot().frame("L_SHOULDER_Y_LINK"), m_stiffness, m_weight / 2);
    m_leftElbowOrientationTask->selectActiveJoints(ctl.solver(), {"L_SHOULDER_Y"});
    m_leftElbowOrientationTask->orientation(Eigen::Quaterniond(0.99144486, 0.0, -0.13052619, 0.).toRotationMatrix());

    ctl.solver().addTask(m_leftElbowOrientationTask);

    m_rightGripperTask =
            std::make_shared<mc_tasks::TransformTask>("RightHandSupportPlate", ctl.robots(), 0, m_stiffness, m_weight);
    m_rightGripperTask->selectActiveJoints(ctl.solver(), RightArmJoints);

    m_rightElbowOrientationTask = std::make_shared<mc_tasks::OrientationTask>(
            ctl.robot().frame("R_SHOULDER_Y_LINK"), m_stiffness, m_weight / 2);
    m_rightElbowOrientationTask->selectActiveJoints(ctl.solver(), {"R_SHOULDER_Y"});
    m_rightElbowOrientationTask->orientation(Eigen::Quaterniond(0.99144486, 0.0, 0.13052619, 0.).toRotationMatrix());

    ctl.solver().addTask(m_rightElbowOrientationTask);

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

    m_BoxHalfWidth = 0.5 *
            (ctl.robot(m_objectName).frame(m_objectSurfaceLeftGripper).position().translation() -
             ctl.robot(m_objectName).frame(m_objectSurfaceRightGripper).position().translation())
                    .norm();


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

        mc_rtc::log::info("Now in raise hands phase");
        m_phase = Phase::RaiseHands;

        m_StartTime = ctl.t();

        m_leftGripperTask->target(
                ctl.robot().frame(m_robotReferenceFrame), {m_leftRaiseOrientationRobot, m_leftRaisePositionRobot});
        ctl.solver().addTask(m_leftGripperTask);

        m_rightGripperTask->target(
                ctl.robot().frame(m_robotReferenceFrame), {m_rightRaiseOrientationRobot, m_rightRaisePositionRobot});
        ctl.solver().addTask(m_rightGripperTask);

        return false;
    }

    bool completed =
            (m_leftGripperTask->eval().norm() < m_completionEval &&
             m_leftGripperTask->speed().norm() < m_completionSpeed &&
             m_rightGripperTask->eval().norm() < m_completionEval &&
             m_rightGripperTask->speed().norm() < m_completionSpeed);

    if (m_phase == Phase::RaiseHands && m_StartTime + m_Timeout < ctl.t())
    {
        mc_rtc::log::info("raise hands timed out");
        completed = true;
    }

    if (m_phase == Phase::RaiseHands && completed)
    {
        if (!m_allowPhaseChange) return false;
        if (m_manualPhaseChange) m_allowPhaseChange = false;

        mc_rtc::log::info("Now in approach phase");
        m_phase = Phase::ApproachBox;

        // set to max double to deactivate the timeout
        m_StartTime = std::numeric_limits<double>::max();

        m_leftGripperTask->targetSurface(
                ctl.robot(m_objectName).robotIndex(),
                m_objectSurfaceLeftGripper,
                {m_leftCarryOrientationRobot, (m_leftApproachOffset + m_leftGraspOffset).eval()});

        m_rightGripperTask->targetSurface(
                ctl.robot(m_objectName).robotIndex(),
                m_objectSurfaceRightGripper,
                {m_rightCarryOrientationRobot, (m_rightApproachOffset + m_rightGraspOffset).eval()});

        return false;
    }

    if (m_phase == Phase::ApproachBox && completed)
    {
        if (!m_allowPhaseChange) return false;
        if (m_manualPhaseChange) m_allowPhaseChange = false;

        mc_rtc::log::info("Now in grasping phase");
        m_phase = Phase::GraspBox;

        m_leftGripperTask->targetSurface(
                ctl.robot(m_objectName).robotIndex(),
                m_objectSurfaceLeftGripper,
                {m_leftCarryOrientationRobot, m_leftGraspOffset});

        m_rightGripperTask->targetSurface(
                ctl.robot(m_objectName).robotIndex(),
                m_objectSurfaceRightGripper,
                {m_rightCarryOrientationRobot, m_rightGraspOffset});

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
                ctl.robot().frame(m_robotReferenceFrame), {m_leftCarryOrientationRobot, m_leftCarryPositionRobot});

        m_rightGripperTask->target(
                ctl.robot().frame(m_robotReferenceFrame), {m_rightCarryOrientationRobot, m_rightCarryPositionRobot});

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
    m_leftGripperTask.reset();
    m_rightGripperTask.reset();

    if (m_contactAdded && m_removeContactAtTeardown)
    {
        ctl.removeContact(m_leftContact);
        ctl.removeContact(m_rightContact);
        m_contactAdded = false;
    }

    ctl.gui()->removeElement({"GraspMoveBox"}, "Next Phase");
    ctl.gui()->removeElement({"GraspMoveBox"}, "Left gripper distance to box and speed");
    ctl.gui()->removeElement({"GraspMoveBox"}, "Right gripper distance to box and speed");
}

EXPORT_SINGLE_STATE("PickupBox", PickupBox)
