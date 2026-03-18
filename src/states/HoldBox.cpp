#include "HoldBox.hpp"
#include <mc_control/Contact.h>
#include <mc_rtc/gui/ArrayInput.h>
#include <mc_rtc/gui/ArrayLabel.h>
#include <mc_rtc/gui/ComboInput.h>
#include <mc_rtc/gui/Label.h>
#include <mc_rtc/gui/NumberInput.h>
#include <mc_rtc/logging.h>
#include "../DemoController.h"
#include "./utils.h"

void HoldBox::configure(const mc_rtc::Configuration &config)
{
    mc_rtc::log::info("HoldBox:\n{}", config.dump(true, true));

    config("robotReferenceFrame", m_robotReferenceFrame);
    config("objectName", m_objectName);
    config("objectSurfaceLeftGripper", m_objectSurfaceLeftGripper);
    config("objectSurfaceRightGripper", m_objectSurfaceRightGripper);
    config("gripperSurfaceLeftGripper", m_gripperSurfaceLeftGripper);
    config("gripperSurfaceRightGripper", m_gripperSurfaceRightGripper);
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

        if (c.r1 == ctl.robot().name() && c.r1Surface == m_gripperSurfaceLeftGripper && c.r2 == m_objectName &&
            c.r2Surface == m_objectSurfaceLeftGripper)
            hasLeftContact = true;

        if (c.r1 == ctl.robot().name() && c.r1Surface == m_gripperSurfaceRightGripper && c.r2 == m_objectName &&
            c.r2Surface == m_objectSurfaceRightGripper)
            hasRightContact = true;
    }

    if (!(hasLeftContact && hasRightContact)) mc_rtc::log::error("Didn't find box contacts");

    m_boxHalfWidth = 0.5 *
            (ctl.robot(m_objectName).frame(m_objectSurfaceLeftGripper).position().translation() -
             ctl.robot(m_objectName).frame(m_objectSurfaceRightGripper).position().translation())
                    .norm();

    m_leftPositionRobot.y()  = m_boxHalfWidth;
    m_rightPositionRobot.y() = -m_boxHalfWidth;

    rebuildTasks(ctl_);

    addToGui(ctl_);
}

bool HoldBox::run(mc_control::fsm::Controller &ctl_)
{
    auto &ctl = static_cast<DemoController &>(ctl_);

    if (m_leftGripperTask)
    {
        m_leftGripperTask->positionTask->stiffness(m_stiffness);
        m_leftGripperTask->positionTask->weight(m_weight);
        m_leftGripperTask->orientationTask->stiffness(m_stiffness);
        m_leftGripperTask->orientationTask->weight(m_weight);
        m_leftGripperTask->set_ef_pose({m_leftOrientationRobot, m_leftPositionRobot});
    }

    if (m_rightGripperTask)
    {
        m_rightGripperTask->positionTask->stiffness(m_stiffness);
        m_rightGripperTask->positionTask->weight(m_weight);
        m_rightGripperTask->orientationTask->stiffness(m_stiffness);
        m_rightGripperTask->orientationTask->weight(m_weight);
        m_rightGripperTask->set_ef_pose({m_rightOrientationRobot, m_rightPositionRobot});
    }

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

    removeFromGui(ctl_);

    ctl.solver().removeTask(m_leftGripperTask);
    ctl.solver().removeTask(m_rightGripperTask);
}

void HoldBox::addToGui(mc_control::fsm::Controller &ctl_)
{
    auto &ctl = static_cast<DemoController &>(ctl_);

    ctl.gui()->addElement(
            {"GMB", "Hold"},
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
            {"GMB", "Hold"},
            mc_rtc::gui::NumberInput(
                    "Stiffness", [this] { return m_stiffness; }, [this](double value) { m_stiffness = value; }),
            mc_rtc::gui::NumberInput(
                    "Weight", [this] { return m_weight; }, [this](double value) { m_weight = value; }),
            mc_rtc::gui::ArrayInput(
                    "Left hold position robot",
                    [this] { return m_leftPositionRobot; },
                    [this](const Eigen::Vector3d &value) { m_leftPositionRobot = value; }),
            mc_rtc::gui::ArrayInput(
                    "Right hold position robot",
                    [this] { return m_rightPositionRobot; },
                    [this](const Eigen::Vector3d &value) { m_rightPositionRobot = value; }));

    ctl.gui()->addElement(
            {"GMB", "Hold"},
            mc_rtc::gui::ComboInput(
                    "Robot reference frame",
                    ctl.robot().frames(),
                    [this] { return m_robotReferenceFrame; },
                    [this, &ctl_](const std::string &frame)
                    {
                        if (frame == m_robotReferenceFrame) return;
                        m_robotReferenceFrame = frame;
                        rebuildTasks(ctl_);
                    }),
            mc_rtc::gui::Label("Object name: ", [this] { return m_objectName; }),
            mc_rtc::gui::Label("Object left surface: ", [this] { return m_objectSurfaceLeftGripper; }),
            mc_rtc::gui::Label("Object right surface: ", [this] { return m_objectSurfaceRightGripper; }),
            mc_rtc::gui::Label("Box half width: ", [this] { return std::to_string(m_boxHalfWidth); }),
            mc_rtc::gui::ArrayLabel("Left orientation robot", [this] { return m_leftOrientationRobot; }),
            mc_rtc::gui::ArrayLabel("Right orientation robot", [this] { return m_rightOrientationRobot; }));
}

void HoldBox::removeFromGui(mc_control::fsm::Controller &ctl_)
{
    auto &ctl = static_cast<DemoController &>(ctl_);
    ctl.gui()->removeCategory({"GMB", "Hold"});
}

void HoldBox::rebuildTasks(mc_control::fsm::Controller &ctl_)
{
    auto &ctl = static_cast<DemoController &>(ctl_);

    if (m_leftGripperTask) ctl.solver().removeTask(m_leftGripperTask);
    if (m_rightGripperTask) ctl.solver().removeTask(m_rightGripperTask);

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

EXPORT_SINGLE_STATE("HoldBox", HoldBox)
