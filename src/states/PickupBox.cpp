#include "PickupBox.hpp"
#include "../DemoController.h"
#include "utils.h"

#include <BaselineWalkingController/CentroidalManager.h>
#include <mc_tasks/CoMTask.h>

void PickupBox::configure(const mc_rtc::Configuration &config)
{
    mc_rtc::log::info("PickupBox:\n{}", config.dump(true, true));

    config("robotReferenceFrame", m_robotReferenceFrame);
    config("objectName", m_objectName);
    config("objectSurfaceLeftGripper", m_objectSurfaceLeftGripper);
    config("objectSurfaceRightGripper", m_objectSurfaceRightGripper);
    config("stiffness", m_stiffness);
    config("weight", m_weight);
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
    config("leftOrientationRobot", m_leftOrientationRobot);
    config("rightOrientationRobot", m_rightOrientationRobot);
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

    m_leftCarryPositionRobot.y()  = m_boxHalfWidth;
    m_rightCarryPositionRobot.y() = -m_boxHalfWidth;

    addToGui(ctl_);
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
    else
    {
        const auto getPosWCall = m_objectName + "::GetPosW";
        if (ctl.datastore().has(getPosWCall))
        {
            const auto &objectPosW = ctl.datastore().call<sva::PTransformd>(getPosWCall);
            ctl.robot(m_objectName).posW(objectPosW);
        }
    }

    m_leftGraspOffsetRobot  = {0.0, m_leftGripperContactOffset, 0.0};
    m_leftGraspOffsetBox    = {0.0, 0.0, m_leftGripperContactOffset};
    m_rightGraspOffsetRobot = {0.0, -m_rightGripperContactOffset, 0.0};
    m_rightGraspOffsetBox   = {0.0, 0.0, m_rightGripperContactOffset};

    m_leftApproachOffsetBox  = {0.0, 0.0, m_approachOffset};
    m_rightApproachOffsetBox = {0.0, 0.0, m_approachOffset};

    if (m_leftGripperTask)
    {
        m_leftGripperTask->stiffness(m_stiffness);
        m_leftGripperTask->weight(m_weight);
    }
    if (m_rightGripperTask)
    {
        m_rightGripperTask->stiffness(m_stiffness);
        m_rightGripperTask->weight(m_weight);
    }

    const bool taskCompleted = m_leftGripperTask->eval().norm() < m_completionEval &&
            m_leftGripperTask->speed().norm() < m_completionSpeed &&
            m_rightGripperTask->eval().norm() < m_completionEval &&
            m_rightGripperTask->speed().norm() < m_completionSpeed;
    bool goToNextPhase = m_phaseAdvanceRequested || (!m_manualPhaseChange && taskCompleted);

    if (m_phase == Phase::None && (!m_manualPhaseChange || m_phaseAdvanceRequested))
    {
        m_phaseAdvanceRequested = false;

        mc_rtc::log::info("Now in approach phase");
        m_phase = Phase::ApproachBox;

        m_comZChanged = false;
    }

    if (m_phase == Phase::ApproachBox)
    {
        if (goToNextPhase)
        {
            m_phaseAdvanceRequested = false;

            mc_rtc::log::info("Now in grasping phase");
            m_phase       = Phase::GraspBox;
            goToNextPhase = false;
        }

        m_leftGripperTask->targetSurface(
                ctl.robot(m_objectName).robotIndex(),
                m_objectSurfaceLeftGripper,
                {m_leftOrientationBox, (m_leftApproachOffsetBox + m_leftGraspOffsetBox).eval()});

        m_rightGripperTask->targetSurface(
                ctl.robot(m_objectName).robotIndex(),
                m_objectSurfaceRightGripper,
                {m_rightOrientationBox, (m_rightApproachOffsetBox + m_rightGraspOffsetBox).eval()});

        if (!m_comZChanged)
        {
            ctl.centroidalManager_->setRefComZ(ctl.m_refCoMZ - m_crouchOffset, ctl.t(), m_crouchOffset * 20.0);
            m_comZChanged = true;
        }
    }

    if (m_phase == Phase::GraspBox)
    {
        if (goToNextPhase)
        {
            m_phaseAdvanceRequested = false;

            mc_rtc::log::info("Now in lift phase");
            m_phase       = Phase::RaiseBox;
            goToNextPhase = false;

            m_comZChanged = false;

            ctl.addContact(m_leftContact);
            ctl.addContact(m_rightContact);
            m_contactAdded = true;
        }

        m_leftGripperTask->targetSurface(
                ctl.robot(m_objectName).robotIndex(),
                m_objectSurfaceLeftGripper,
                {m_leftOrientationBox, m_leftGraspOffsetBox});

        m_rightGripperTask->targetSurface(
                ctl.robot(m_objectName).robotIndex(),
                m_objectSurfaceRightGripper,
                {m_rightOrientationBox, m_rightGraspOffsetBox});
    }

    if (m_phase == Phase::RaiseBox)
    {
        if (goToNextPhase)
        {
            m_phaseAdvanceRequested = false;

            output("OK");
            return true;
        }

        m_leftGripperTask->target(
                ctl.robot().frame(m_robotReferenceFrame),
                {m_leftOrientationRobot, m_leftCarryPositionRobot + m_leftGraspOffsetRobot});

        m_rightGripperTask->target(
                ctl.robot().frame(m_robotReferenceFrame),
                {m_rightOrientationRobot, m_rightCarryPositionRobot + m_rightGraspOffsetRobot});

        if (!m_comZChanged)
        {
            ctl.centroidalManager_->setRefComZ(ctl.m_refCoMZ, ctl.t(), m_crouchOffset * 20.0);
            m_comZChanged = true;
        }
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

    removeFromGui(ctl_);
}

void PickupBox::addToGui(mc_control::fsm::Controller &ctl_)
{
    auto &ctl = static_cast<DemoController &>(ctl_);

    auto boolToString = [](bool value) { return value ? "True" : "False"; };

    ctl.gui()->addElement(
            {"GMB", "Pickup"},
            mc_rtc::gui::Label(
                    "Left gripper task eval norm: ",
                    [this]
                    {
                        std::string data = std::to_string(m_leftGripperTask->eval().norm());
                        data += "m\t";
                        data += std::to_string(m_leftGripperTask->speed().norm());
                        data += "m/s";
                        return data;
                    }),
            mc_rtc::gui::Label(
                    "Right gripper task eval norm: ",
                    [this]
                    {
                        std::string data = std::to_string(m_rightGripperTask->eval().norm());
                        data += "m\t";
                        data += std::to_string(m_rightGripperTask->speed().norm());
                        data += "m/s";
                        return data;
                    }),
            mc_rtc::gui::Label("Current CoM Z: ", [&ctl] { return std::to_string(ctl.robot().com().z()); }));

    ctl.gui()->addElement(
            {"GMB", "Pickup"},
            mc_rtc::gui::Button(
                    "Force Add Contacts",
                    [&ctl, this]
                    {
                        ctl.addContact(m_leftContact);
                        ctl.addContact(m_rightContact);
                    }));

    ctl.gui()->addElement(
            {"GMB", "Pickup"},
            mc_rtc::gui::Button("Next Phase", [this] { m_phaseAdvanceRequested = true; }),
            mc_rtc::gui::Label(
                    "Current Phase: ",
                    [this]
                    {
                        switch (m_phase)
                        {
                            case Phase::None: return std::string{"None"};
                            case Phase::ApproachBox: return std::string{"ApproachBox"};
                            case Phase::GraspBox: return std::string{"GraspBox"};
                            case Phase::RaiseBox: return std::string{"RaiseBox"};
                        }
                        return std::string{"Unknown"};
                    }),
            mc_rtc::gui::Checkbox(
                    "Manual phase change: ",
                    [this] { return m_manualPhaseChange; },
                    [this] { m_manualPhaseChange = !m_manualPhaseChange; }));

    ctl.gui()->addElement(
            {"GMB", "Pickup"},
            mc_rtc::gui::NumberInput(
                    "Left gripper contact offset",
                    [this] { return m_leftGripperContactOffset; },
                    [this](double value) { m_leftGripperContactOffset = value; }),
            mc_rtc::gui::NumberInput(
                    "Right gripper contact offset",
                    [this] { return m_rightGripperContactOffset; },
                    [this](double value) { m_rightGripperContactOffset = value; }),
            mc_rtc::gui::NumberInput(
                    "Approach offset",
                    [this] { return m_approachOffset; },
                    [this](double value) { m_approachOffset = value; }),
            mc_rtc::gui::NumberInput(
                    "Crouch offset",
                    [this] { return m_crouchOffset; },
                    [this](double value)
                    {
                        m_crouchOffset = value;
                        m_comZChanged  = false;
                    }),
            mc_rtc::gui::NumberInput(
                    "Stiffness", [this] { return m_stiffness; }, [this](double value) { m_stiffness = value; }),
            mc_rtc::gui::NumberInput(
                    "Weight", [this] { return m_weight; }, [this](double value) { m_weight = value; }),
            mc_rtc::gui::ArrayInput(
                    "Left carry position robot",
                    [this] { return m_leftCarryPositionRobot; },
                    [this](const std::vector<double> &values)
                    {
                        if (values.size() == 3) m_leftCarryPositionRobot = Eigen::Vector3d(values.data());
                    }),
            mc_rtc::gui::ArrayInput(
                    "Right carry position robot",
                    [this] { return m_rightCarryPositionRobot; },
                    [this](const std::vector<double> &values)
                    {
                        if (values.size() == 3) m_rightCarryPositionRobot = Eigen::Vector3d(values.data());
                    }));

    ctl.gui()->addElement(
            {"GMB", "Pickup"},
            mc_rtc::gui::Label("Robot reference frame: ", [this] { return m_robotReferenceFrame; }),
            mc_rtc::gui::Label("Object name: ", [this] { return m_objectName; }),
            mc_rtc::gui::Label("Object left surface: ", [this] { return m_objectSurfaceLeftGripper; }),
            mc_rtc::gui::Label("Object right surface: ", [this] { return m_objectSurfaceRightGripper; }),
            mc_rtc::gui::Label("Gripper left surface: ", [this] { return m_gripperSurfaceLeftGripper; }),
            mc_rtc::gui::Label("Gripper right surface: ", [this] { return m_gripperSurfaceRightGripper; }),
            mc_rtc::gui::Label("Completion eval: ", [this] { return std::to_string(m_completionEval); }),
            mc_rtc::gui::Label("Completion speed: ", [this] { return std::to_string(m_completionSpeed); }),
            mc_rtc::gui::Label(
                    "Contact added: ", [this, &boolToString] { return std::string{boolToString(m_contactAdded)}; }),
            mc_rtc::gui::Label(
                    "Remove contact at teardown: ",
                    [this, &boolToString] { return std::string{boolToString(m_removeContactAtTeardown)}; }),
            mc_rtc::gui::ArrayLabel("Left grasp offset box", [this] { return m_leftGraspOffsetBox; }),
            mc_rtc::gui::ArrayLabel("Right grasp offset box", [this] { return m_rightGraspOffsetBox; }),
            mc_rtc::gui::ArrayLabel("Left approach offset box", [this] { return m_leftApproachOffsetBox; }),
            mc_rtc::gui::ArrayLabel("Right approach offset box", [this] { return m_rightApproachOffsetBox; }),
            mc_rtc::gui::ArrayLabel("Left grasp offset robot", [this] { return m_leftGraspOffsetRobot; }),
            mc_rtc::gui::ArrayLabel("Right grasp offset robot", [this] { return m_rightGraspOffsetRobot; }),
            mc_rtc::gui::ArrayLabel("Left orientation box", [this] { return m_leftOrientationBox; }),
            mc_rtc::gui::ArrayLabel("Right orientation box", [this] { return m_rightOrientationBox; }),
            mc_rtc::gui::ArrayLabel("Left orientation robot", [this] { return m_leftOrientationRobot; }),
            mc_rtc::gui::ArrayLabel("Right orientation robot", [this] { return m_rightOrientationRobot; }));
}

void PickupBox::removeFromGui(mc_control::fsm::Controller &ctl_)
{
    auto &ctl = static_cast<DemoController &>(ctl_);
    ctl.gui()->removeCategory({"GMB", "Pickup"});
}

EXPORT_SINGLE_STATE("PickupBox", PickupBox)
