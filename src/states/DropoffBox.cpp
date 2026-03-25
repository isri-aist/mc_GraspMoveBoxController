#include "DropoffBox.hpp"

#include <BaselineWalkingController/CentroidalManager.h>
#include <BaselineWalkingController/FootManager.h>
#include <Eigen/Geometry>
#include <mc_rtc/gui/ArrayLabel.h>
#include <mc_rtc/gui/Checkbox.h>
#include <mc_rtc/gui/Label.h>
#include <mc_rtc/gui/NumberInput.h>
#include <mc_tasks/CoMTask.h>
#include "../DemoController.h"
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
    config("completionEval", m_completionEval);
    config("completionSpeed", m_completionSpeed);
    config("crouchOffset", m_crouchOffset);
    config("removeContactsAtTeardown", m_removeContactAtTeardown);
    config("manualPhaseChange", m_manualPhaseChange);
    config("leftGripperContactOffset", m_leftGripperContactOffset);
    config("rightGripperContactOffset", m_rightGripperContactOffset);
    config("leftApproachOffsetRobot", m_leftApproachOffsetRobot);
    config("rightApproachOffsetRobot", m_rightApproachOffsetRobot);
    config("leftDropPositionRobot", m_leftDropPositionRobot);
    config("rightDropPositionRobot", m_rightDropPositionRobot);
    config("leftOrientationBox", m_leftOrientationBox);
    config("rightOrientationBox", m_rightOrientationBox);
    config("leftOrientationRobot", m_leftOrientationRobot);
    config("rightOrientationRobot", m_rightOrientationRobot);

    m_phaseAdvanceRequested = false;
}

void DropoffBox::start(mc_control::fsm::Controller &ctl_)
{
    auto &ctl = static_cast<DemoController &>(ctl_);

    bool hasLeftContact = false, hasRightContact = false;

    for (const auto &c : ctl.contacts())
    {
        mc_rtc::log::info("contact: {}:{} <-> {}:{}", c.r1->c_str(), c.r1Surface, c.r2->c_str(), c.r2Surface);

        if (c.r1 == ctl.robot().name() && c.r1Surface == m_gripperSurfaceLeftGripper && c.r2 == m_objectName &&
            c.r2Surface == m_objectSurfaceLeftGripper)
            hasLeftContact = true;

        if (c.r1 == ctl.robot().name() && c.r1Surface == m_gripperSurfaceRightGripper && c.r2 == m_objectName &&
            c.r2Surface == m_objectSurfaceRightGripper)
            hasRightContact = true;
    }
    if (!hasLeftContact || !hasRightContact) mc_rtc::log::error("Didn't find box contacts");
    m_contactAdded = true;

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

    const double handDistance = (ctl.robot().frame(m_gripperSurfaceLeftGripper).position().translation() -
                                 ctl.robot().frame(m_gripperSurfaceRightGripper).position().translation())
                                        .norm();

    m_boxHalfWidth = 0.5 * handDistance;

    m_leftDropPositionRobot.y()  = m_boxHalfWidth;
    m_rightDropPositionRobot.y() = -m_boxHalfWidth;

    addToGui(ctl);
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
    else
    {
        const auto getPosWCall = m_objectName + "::GetPosW";
        if (ctl.datastore().has(getPosWCall))
        {
            const auto &objectPosW = ctl.datastore().call<sva::PTransformd>(getPosWCall);
            ctl.robot(m_objectName).posW(objectPosW);
        }
    }

    handlePhaseChange(ctl);
    updateStateConfig(ctl);

    if (m_phase == Phase::Finished)
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

    removeFromGui(ctl);
}

void DropoffBox::handlePhaseChange(DemoController &ctl)
{
    bool taskCompleted = m_leftGripperTask->eval().norm() < m_completionEval &&
            m_leftGripperTask->speed().norm() < m_completionSpeed &&
            m_rightGripperTask->eval().norm() < m_completionEval &&
            m_rightGripperTask->speed().norm() < m_completionSpeed;

    if (m_phase == Phase::Init) taskCompleted = true;

    bool goToNextPhase = m_phaseAdvanceRequested || (!m_manualPhaseChange && taskCompleted);

    if (!goToNextPhase) return;

    std::string currentPhaseName;

    switch (m_phase)
    {
        case Phase::Init:
            m_phase          = Phase::LowerBox;
            currentPhaseName = "LowerBox";
            break;
        case Phase::LowerBox:
            if (m_contactAdded)
            {
                ctl.clearContacts();
                m_contactAdded = false;
            }
            // reset box position because it doesn't have physics and stays in mid air
            // TODO: find a better fix
            ctl.robot(m_objectName).posW(ctl.config()("robots")(m_objectName)("init_pos"));

            m_phase          = Phase::DropBox;
            currentPhaseName = "DropBox";
            break;
        case Phase::DropBox:
            m_phase          = Phase::Retreat;
            currentPhaseName = "Retreat";
            break;
        case Phase::Retreat:
            m_phase          = Phase::Finished;
            currentPhaseName = "Finished";
            break;
        case Phase::Finished: break;
    }

    m_phaseAdvanceRequested    = false;
    m_centroidManagerDidItsJob = false;

    const char *statusColor = taskCompleted ? "\x1b[30;42m" : "\x1b[37;41m";

    mc_rtc::log::info(
            "Phase changed to {} ({}eval: {}\x1b[0m, request: {})",
            currentPhaseName,
            statusColor,
            taskCompleted,
            m_phaseAdvanceRequested);
}

void DropoffBox::updateStateConfig(DemoController &ctl)
{
    m_leftGraspOffsetRobot  = {0.0, std::abs(m_leftGripperContactOffset), 0.0};
    m_leftGraspOffsetBox    = {0.0, 0.0, m_leftGripperContactOffset};
    m_rightGraspOffsetRobot = {0.0, -std::abs(m_rightGripperContactOffset), 0.0};
    m_rightGraspOffsetBox   = {0.0, 0.0, m_rightGripperContactOffset};

    m_leftApproachOffsetBox  = BoxOffsetFromRobotOffset(m_leftApproachOffsetRobot, BoxNoLid, Left);
    m_rightApproachOffsetBox = BoxOffsetFromRobotOffset(m_rightApproachOffsetRobot, BoxNoLid, Right);

    m_leftGripperTask->stiffness(m_stiffness);
    m_leftGripperTask->weight(m_weight);

    m_rightGripperTask->stiffness(m_stiffness);
    m_rightGripperTask->weight(m_weight);

    switch (m_phase)
    {
        case (Phase::Init):
        case (Phase::Finished): break;

        case (Phase::LowerBox):
        {
            m_leftGripperTask->target(
                    ctl.robot().frame(m_robotReferenceFrame), {m_leftOrientationRobot, m_leftDropPositionRobot});
            m_rightGripperTask->target(
                    ctl.robot().frame(m_robotReferenceFrame), {m_rightOrientationRobot, m_rightDropPositionRobot});

            if (!m_centroidManagerDidItsJob)
                m_centroidManagerDidItsJob = ctl.centroidalManager_->setRefComZ(
                        ctl.m_refCoMZ - m_crouchOffset, ctl.t() + 1.0, m_crouchOffset * 30.0);

            break;
        }

        case (Phase::DropBox):
        {
            auto leftOffsetRotation  = sva::RotZ(-mc_rtc::constants::PI / 2);
            auto rightOffsetRotation = sva::RotZ(mc_rtc::constants::PI / 2);

            m_leftGripperTask->targetSurface(
                    ctl.robot(m_objectName).robotIndex(),
                    m_objectSurfaceLeftGripper,
                    {leftOffsetRotation * m_leftOrientationBox,
                     (m_leftApproachOffsetBox + m_leftGraspOffsetBox).eval()});

            m_rightGripperTask->targetSurface(
                    ctl.robot(m_objectName).robotIndex(),
                    m_objectSurfaceRightGripper,
                    {rightOffsetRotation * m_rightOrientationBox,
                     (m_rightApproachOffsetBox + m_rightGraspOffsetBox).eval()});
            break;
        }

        case (Phase::Retreat):
        {
            m_leftGripperTask->target(
                    ctl.robot().frame(m_robotReferenceFrame),
                    {ctl.robot().frame(m_gripperSurfaceLeftGripper).position().rotation(),
                     m_leftDropPositionRobot + m_leftGraspOffsetRobot + m_leftApproachOffsetRobot +
                             Eigen::Vector3d{0.0, 0.0, m_crouchOffset}});
            m_rightGripperTask->target(
                    ctl.robot().frame(m_robotReferenceFrame),
                    {ctl.robot().frame(m_gripperSurfaceRightGripper).position().rotation(),
                     m_rightDropPositionRobot + m_rightGraspOffsetRobot + m_rightApproachOffsetRobot +
                             Eigen::Vector3d{0.0, 0.0, m_crouchOffset}});

            if (!m_centroidManagerDidItsJob)
                m_centroidManagerDidItsJob =
                        ctl.centroidalManager_->setRefComZ(ctl.m_refCoMZ, ctl.t() + 1.0, m_crouchOffset * 30.0);
            break;
        }
    }
}

void DropoffBox::addToGui(DemoController &ctl)
{
    auto boolToString = [](bool value) { return value ? "True" : "False"; };

    ctl.gui()->addElement(
            {"GMB", "Dropoff"},
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
                    }));

    ctl.gui()->addElement(
            {"GMB", "Dropoff"},
            mc_rtc::gui::Button("Next Phase", [this] { m_phaseAdvanceRequested = true; }),
            mc_rtc::gui::Label(
                    "Current Phase: ",
                    [this]
                    {
                        switch (m_phase)
                        {
                            case Phase::Init: return std::string{"None"};
                            case Phase::LowerBox: return std::string{"LowerBox"};
                            case Phase::DropBox: return std::string{"DropBox"};
                            case Phase::Retreat: return std::string{"Retreat"};
                        }
                        return std::string{"Unknown"};
                    }),
            mc_rtc::gui::Checkbox(
                    "Manual phase change: ",
                    [this] { return m_manualPhaseChange; },
                    [this] { m_manualPhaseChange = !m_manualPhaseChange; }));


    ctl.gui()->addElement(
            {"GMB", "Dropoff"},
            mc_rtc::gui::NumberInput(
                    "Left gripper contact offset",
                    [this] { return m_leftGripperContactOffset; },
                    [this](double value) { m_leftGripperContactOffset = value; }),
            mc_rtc::gui::NumberInput(
                    "Right gripper contact offset",
                    [this] { return m_rightGripperContactOffset; },
                    [this](double value) { m_rightGripperContactOffset = value; }),
            mc_rtc::gui::NumberInput(
                    "Crouch offset",
                    [this] { return m_crouchOffset; },
                    [this](double value)
                    {
                        m_centroidManagerDidItsJob = false;
                        m_crouchOffset             = value;
                    }),
            mc_rtc::gui::NumberInput(
                    "Stiffness", [this] { return m_stiffness; }, [this](double value) { m_stiffness = value; }),
            mc_rtc::gui::NumberInput(
                    "Weight", [this] { return m_weight; }, [this](double value) { m_weight = value; }),
            mc_rtc::gui::ArrayInput(
                    "Left approach offset robot",
                    [this] { return m_leftApproachOffsetRobot; },
                    [this](const Eigen::Vector3d &value)
                    {
                        m_leftApproachOffsetRobot = value;
                        m_leftApproachOffsetBox   = BoxOffsetFromRobotOffset(m_leftApproachOffsetRobot, BoxNoLid, Left);
                    }),
            mc_rtc::gui::ArrayInput(
                    "Right approach offset robot",
                    [this] { return m_rightApproachOffsetRobot; },
                    [this](const Eigen::Vector3d &value)
                    {
                        m_rightApproachOffsetRobot = value;
                        m_rightApproachOffsetBox =
                                BoxOffsetFromRobotOffset(m_rightApproachOffsetRobot, BoxNoLid, Right);
                    }),
            mc_rtc::gui::ArrayInput(
                    "Left drop position robot",
                    [this] { return m_leftDropPositionRobot; },
                    [this](const Eigen::Vector3d &value) { m_leftDropPositionRobot = value; }),
            mc_rtc::gui::ArrayInput(
                    "Right drop position robot",
                    [this] { return m_rightDropPositionRobot; },
                    [this](const Eigen::Vector3d &value) { m_rightDropPositionRobot = value; }));

    ctl.gui()->addElement(
            {"GMB", "Dropoff"},
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

void DropoffBox::removeFromGui(DemoController &ctl)
{
    ctl.gui()->removeCategory({"GMB", "Dropoff"});
}


EXPORT_SINGLE_STATE("DropoffBox", DropoffBox)
