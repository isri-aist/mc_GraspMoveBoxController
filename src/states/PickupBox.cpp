#include "PickupBox.hpp"

#include <BaselineWalkingController/CentroidalManager.h>
#include <mc_tasks/CoMTask.h>
#include "../DemoController.h"
#include "utils.h"


void PickupBox::configure(const mc_rtc::Configuration &config)
{
    mc_rtc::log::info("PickupBox:\n{}", config.dump(true, true));

    config("robotReferenceFrame", m_robotReferenceFrame);
    config("objectName", m_objectName);
    config("objectSurfaceLeftGripper", m_objectSurfaceLeftGripper);
    config("objectSurfaceRightGripper", m_objectSurfaceRightGripper);
    config("gripperSurfaceLeftGripper", m_gripperSurfaceLeftGripper);
    config("gripperSurfaceRightGripper", m_gripperSurfaceRightGripper);
    config("stiffness", m_stiffness);
    config("weight", m_weight);
    config("admittanceStiffness", m_admittanceStiffness);
    config("admittanceDamping", m_admittanceDamping);
    config("leftAdmittanceWrenchTarget", m_leftAdmittanceWrenchTarget);
    config("rightAdmittanceWrenchTarget", m_rightAdmittanceWrenchTarget);
    config("admittanceCoefficient", m_admittanceCoefficient);
    config("completionEval", m_completionEval);
    config("completionSpeed", m_completionSpeed);
    config("removeContactsAtTeardown", m_removeContactAtTeardown);
    config("manualPhaseChange", m_manualPhaseChange);
    config("leftGripperContactOffset", m_leftGripperContactOffset);
    config("rightGripperContactOffset", m_rightGripperContactOffset);
    config("leftApproachOffsetRobot", m_leftApproachOffsetRobot);
    config("rightApproachOffsetRobot", m_rightApproachOffsetRobot);
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

    // for (const auto &r : ctl.robots()) mc_rtc::log::info("{}", r.name());

    m_leftGripperTask = std::make_shared<mc_tasks::force::AdmittanceTask>(
            m_gripperSurfaceLeftGripper, ctl.robots(), 0, m_stiffness, m_weight);
    m_leftGripperTask->selectActiveJoints(ctl.solver(), LeftArmJoints);
    m_leftGripperTask->targetPose(ctl.robot().frame(m_gripperSurfaceLeftGripper).position());
    ctl.solver().addTask(m_leftGripperTask);

    m_rightGripperTask = std::make_shared<mc_tasks::force::AdmittanceTask>(
            m_gripperSurfaceRightGripper, ctl.robots(), 0, m_stiffness, m_weight);
    m_rightGripperTask->selectActiveJoints(ctl.solver(), RightArmJoints);
    m_rightGripperTask->targetPose(ctl.robot().frame(m_gripperSurfaceRightGripper).position());
    ctl.solver().addTask(m_rightGripperTask);

    m_leftContact = mc_control::Contact(
            ctl.robot().name(),
            ctl.robot(m_objectName).name(),
            m_gripperSurfaceLeftGripper,
            m_objectSurfaceLeftGripper,
            mc_rbdyn::Contact::defaultFriction,
            Eigen::Vector6d::Ones());

    m_rightContact = mc_control::Contact(
            ctl.robot().name(),
            ctl.robot(m_objectName).name(),
            m_gripperSurfaceRightGripper,
            m_objectSurfaceRightGripper,
            mc_rbdyn::Contact::defaultFriction,
            Eigen::Vector6d::Ones());

    m_boxHalfWidth = 0.5 *
            (ctl.robot(m_objectName).frame(m_objectSurfaceLeftGripper).position().translation() -
             ctl.robot(m_objectName).frame(m_objectSurfaceRightGripper).position().translation())
                    .norm();

    m_leftCarryPositionRobot.y()  = m_boxHalfWidth;
    m_rightCarryPositionRobot.y() = -m_boxHalfWidth;

    m_crouchOffset = ctl.m_refCoMZ - ctl.robot(m_objectName).posW().translation().z() - 0.1;
    addToGui(ctl);
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

    handlePhaseChange(ctl);
    updateStateConfig(ctl);

    if (m_phase == Phase::Finished)
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

    removeFromGui(ctl);
}

void PickupBox::handlePhaseChange(DemoController &ctl)
{
    bool taskCompleted = m_leftGripperTask->eval().norm() < m_completionEval &&
            m_leftGripperTask->speed().norm() < m_completionSpeed &&
            m_rightGripperTask->eval().norm() < m_completionEval &&
            m_rightGripperTask->speed().norm() < m_completionSpeed;

    if (m_phase == Phase::Init) taskCompleted = true;

    if (m_phase == Phase::GraspBox)
        taskCompleted = m_leftGripperTask->measuredWrench().force().x() > m_leftCarryWrench.force().x() &&
                m_rightGripperTask->measuredWrench().force().x() > m_rightCarryWrench.force().x();

    bool goToNextPhase = m_phaseAdvanceRequested || (!m_manualPhaseChange && taskCompleted);

    if (!goToNextPhase) return;

    std::string currentPhaseName;

    switch (m_phase)
    {
        case Phase::Init:
            m_phase          = Phase::ApproachBox;
            currentPhaseName = "ApproachBox";
            break;
        case Phase::ApproachBox:
            m_phase          = Phase::GraspBox;
            currentPhaseName = "GraspBox";
            break;
        case Phase::GraspBox:
            m_leftGripperTask->targetWrench(sva::ForceVecd::Zero());
            m_leftGripperTask->admittance(sva::ForceVecd::Zero());
            m_rightGripperTask->targetWrench(sva::ForceVecd::Zero());
            m_rightGripperTask->admittance(sva::ForceVecd::Zero());
            ctl.addContact(m_leftContact);
            ctl.addContact(m_rightContact);
            m_contactAdded   = true;
            m_phase          = Phase::RaiseBox;
            currentPhaseName = "RaiseBox";
            break;
        case Phase::RaiseBox:
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

void PickupBox::updateStateConfig(DemoController &ctl)
{
    m_dimStiffness =
            Eigen::Vector6d(m_stiffness, m_stiffness, m_stiffness, m_admittanceStiffness, m_stiffness, m_stiffness);
    m_dimDamping = Eigen::Vector6d(6.3, 6.3, 6.3, m_admittanceDamping, 6.3, 6.3);

    m_admittanceCoefficients = {{0, 0, 0}, {m_admittanceCoefficient, 0, 0}};

    m_leftCarryWrench  = {{0.0, 0.0, 0.0}, {m_leftAdmittanceWrenchTarget, 0.0, 0.0}};
    m_rightCarryWrench = {{0.0, 0.0, 0.0}, {m_rightAdmittanceWrenchTarget, 0.0, 0.0}};

    m_leftGraspOffsetRobot  = {0.0, std::abs(m_leftGripperContactOffset), 0.0};
    m_rightGraspOffsetRobot = {0.0, -std::abs(m_rightGripperContactOffset), 0.0};

    m_leftGraspOffsetBox  = {0.0, 0.0, m_leftGripperContactOffset};
    m_rightGraspOffsetBox = {0.0, 0.0, m_rightGripperContactOffset};

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
        case (Phase::ApproachBox):
        {
            m_leftGripperTask->targetSurface(
                    ctl.robot(m_objectName).robotIndex(),
                    m_objectSurfaceLeftGripper,
                    {m_leftOrientationBox, (m_leftApproachOffsetBox + m_leftGraspOffsetBox).eval()});

            m_rightGripperTask->targetSurface(
                    ctl.robot(m_objectName).robotIndex(),
                    m_objectSurfaceRightGripper,
                    {m_rightOrientationBox, (m_rightApproachOffsetBox + m_rightGraspOffsetBox).eval()});

            if (!m_centroidManagerDidItsJob)
                m_centroidManagerDidItsJob = ctl.centroidalManager_->setRefComZ(
                        ctl.m_refCoMZ - m_crouchOffset, ctl.t() + 1.0, m_crouchOffset * 30.0);

            break;
        }
        case (Phase::GraspBox):
        {
            m_leftGripperTask->targetSurface(
                    ctl.robot(m_objectName).robotIndex(),
                    m_objectSurfaceLeftGripper,
                    {m_leftOrientationBox, m_leftGraspOffsetBox});

            m_rightGripperTask->targetSurface(
                    ctl.robot(m_objectName).robotIndex(),
                    m_objectSurfaceRightGripper,
                    {m_rightOrientationBox, m_rightGraspOffsetBox});

            m_leftGripperTask->targetWrench(m_leftCarryWrench);
            m_leftGripperTask->admittance(m_admittanceCoefficients);
            m_leftGripperTask->stiffness(m_dimStiffness);
            m_leftGripperTask->damping(m_dimDamping);

            m_rightGripperTask->targetWrench(m_rightCarryWrench);
            m_rightGripperTask->admittance(m_admittanceCoefficients);
            m_rightGripperTask->stiffness(m_dimStiffness);
            m_rightGripperTask->damping(m_dimDamping);

            break;
        }
        case (Phase::RaiseBox):
        {
            m_leftGripperTask->targetPose(
                    sva::PTransformd{m_leftOrientationRobot, m_leftCarryPositionRobot + m_leftGraspOffsetRobot} *
                    ctl.robot().frame(m_robotReferenceFrame).position());

            m_rightGripperTask->targetPose(
                    sva::PTransformd{m_rightOrientationRobot, m_rightCarryPositionRobot + m_rightGraspOffsetRobot} *
                    ctl.robot().frame(m_robotReferenceFrame).position());

            if (!m_centroidManagerDidItsJob)
                m_centroidManagerDidItsJob =
                        ctl.centroidalManager_->setRefComZ(ctl.m_refCoMZ, ctl.t() + 1.0, m_crouchOffset * 30.0);

            break;
        }
    }
}

void PickupBox::addToGui(DemoController &ctl)
{
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
                            case Phase::Init: return std::string{"Init"};
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
                    "Crouch offset",
                    [this] { return m_crouchOffset; },
                    [this](double value)
                    {
                        m_centroidManagerDidItsJob = false;
                        m_crouchOffset             = value;
                    }),
            mc_rtc::gui::NumberInput(
                    "Left carry wrench robot",
                    [this] { return m_leftAdmittanceWrenchTarget; },
                    [this](double value) { m_leftAdmittanceWrenchTarget = value; }),
            mc_rtc::gui::NumberInput(
                    "Right carry wrench robot",
                    [this] { return m_rightAdmittanceWrenchTarget; },
                    [this](double value) { m_rightAdmittanceWrenchTarget = value; }),
            mc_rtc::gui::NumberInput(
                    "Stiffness", [this] { return m_stiffness; }, [this](double value) { m_stiffness = value; }),
            mc_rtc::gui::NumberInput(
                    "admittanceStiffness",
                    [this] { return m_admittanceStiffness; },
                    [this](double value) { m_admittanceStiffness = value; }),
            mc_rtc::gui::NumberInput(
                    "admittanceDamping",
                    [this] { return m_admittanceDamping; },
                    [this](double value) { m_admittanceDamping = value; }),
            mc_rtc::gui::NumberInput(
                    "admittanceCoefficient",
                    [this] { return m_admittanceCoefficient; },
                    [this](double value) { m_admittanceCoefficient = value; }),
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
                    "Left carry position robot",
                    [this] { return m_leftCarryPositionRobot; },
                    [this](const Eigen::Vector3d &value) { m_leftCarryPositionRobot = value; }),
            mc_rtc::gui::ArrayInput(
                    "Right carry position robot",
                    [this] { return m_rightCarryPositionRobot; },
                    [this](const Eigen::Vector3d &value) { m_rightCarryPositionRobot = value; }));

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

void PickupBox::removeFromGui(DemoController &ctl)
{
    ctl.gui()->removeCategory({"GMB", "Pickup"});
}

EXPORT_SINGLE_STATE("PickupBox", PickupBox)
