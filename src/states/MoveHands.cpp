#include "MoveHands.hpp"
#include "../DemoController.h"
#include "utils.h"

#include <mc_rtc/gui/ArrayInput.h>
#include <mc_rtc/gui/ComboInput.h>
#include <mc_rtc/gui/Label.h>
#include <mc_rtc/gui/NumberInput.h>

void MoveHands::configure(const mc_rtc::Configuration & config)
{
    m_config.load(config);

    mc_rtc::log::info("MoveHands:\n{}", m_config.dump(true, true));

    if (
        !m_config.has("leftHandTargetPositionRobot")
        || !m_config.has("rightHandTargetPositionRobot")
        || !m_config.has("leftHandTargetOrientationRobot")
        || !m_config.has("rightHandTargetOrientationRobot")
    )
        mc_rtc::log::error_and_throw("Configuration is missing fields");

    // assume RHPS1
    m_robotReferenceFrame = m_config("robotReferenceFrame", std::string("CHEST_Y_LINK"));
    m_leftHandFrame       = m_config("leftHandFrame", std::string("LeftHandSupportPlate"));
    m_rightHandFrame      = m_config("rightHandFrame", std::string("RightHandSupportPlate"));

    m_stiffness = m_config("stiffness", 2.0);
    m_weight    = m_config("weight", 1000.0);

    m_autoStart = m_config("autoStart", false);

    m_started = m_autoStart;

    m_config("leftHandTargetPositionRobot", m_leftHandTargetPositionRobot);
    m_config("rightHandTargetPositionRobot", m_rightHandTargetPositionRobot);

    m_config("leftHandTargetOrientationRobot", m_leftHandTargetOrientationRobot);
    m_config("rightHandTargetOrientationRobot", m_rightHandTargetOrientationRobot);
}

void MoveHands::start(mc_control::fsm::Controller & ctl_)
{
    auto & ctl = static_cast<DemoController&>(ctl_);

    m_leftGripperTask =
        std::make_shared<mc_tasks::RelativeEndEffectorTask>(
            m_leftHandFrame,
            ctl.robots(),
            0,
            m_robotReferenceFrame,
            m_stiffness,
            m_weight
            );

    m_rightGripperTask =
        std::make_shared<mc_tasks::RelativeEndEffectorTask>(
            m_rightHandFrame,
            ctl.robots(),
            0,
            m_robotReferenceFrame,
            m_stiffness,
            m_weight
            );

    m_leftGripperTask->selectActiveJoints(ctl.solver(), LeftArmJoints);
    m_rightGripperTask->selectActiveJoints(ctl.solver(), RightArmJoints);

    addToGui(ctl_);
}

bool MoveHands::run(mc_control::fsm::Controller & ctl_)
{
    auto & ctl = static_cast<DemoController&>(ctl_);

    if (m_started)
    {
        ctl.solver().addTask(m_leftGripperTask);
        ctl.solver().addTask(m_rightGripperTask);
    }

    m_leftGripperTask->positionTask->stiffness(m_stiffness);
    m_leftGripperTask->positionTask->weight(m_weight);
    m_leftGripperTask->orientationTask->stiffness(m_stiffness);
    m_leftGripperTask->orientationTask->weight(m_weight);
    m_leftGripperTask->set_ef_pose({m_leftHandTargetOrientationRobot, m_leftHandTargetPositionRobot});

    m_rightGripperTask->positionTask->stiffness(m_stiffness);
    m_rightGripperTask->positionTask->weight(m_weight);
    m_rightGripperTask->orientationTask->stiffness(m_stiffness);
    m_rightGripperTask->orientationTask->weight(m_weight);
    m_rightGripperTask->set_ef_pose({m_rightHandTargetOrientationRobot, m_rightHandTargetPositionRobot});

    return true;
}

void MoveHands::teardown(mc_control::fsm::Controller & ctl_)
{
    auto & ctl = static_cast<DemoController&>(ctl_);

    removeFromGui(ctl_);

    ctl.solver().removeTask(m_leftGripperTask);
    ctl.solver().removeTask(m_rightGripperTask);
}

void MoveHands::addToGui(mc_control::fsm::Controller & ctl_)
{
    auto & ctl = static_cast<DemoController&>(ctl_);

    if (!m_autoStart)
    {
        ctl.gui()->addElement(
            {"GMB", "MoveHands"},
            mc_rtc::gui::Button(
                "Start",
                [this]
                {
                    m_started = true;
                }
                )
            );
    }

    ctl.gui()->addElement(
        {"GMB", "MoveHands"},
        mc_rtc::gui::Label(
            "Left gripper task eval norm: ",
            [this]
            {
                std::string data = std::to_string(m_leftGripperTask->eval().norm());
                data             += "m\t";
                data             += std::to_string(m_leftGripperTask->speed().norm());
                data             += "m/s";
                return data;
            }
            ),
        mc_rtc::gui::Label(
            "Right gripper task eval norm: ",
            [this]
            {
                std::string data = std::to_string(m_rightGripperTask->eval().norm());
                data             += "m\t";
                data             += std::to_string(m_rightGripperTask->speed().norm());
                data             += "m/s";
                return data;
            }
            )
        );

    ctl.gui()->addElement(
        {"GMB", "MoveHands"},
        mc_rtc::gui::NumberInput(
            "Stiffness",
            [this]
            {
                return m_stiffness;
            },
            [this](double value)
            {
                m_stiffness = value;
            }
            ),
        mc_rtc::gui::NumberInput(
            "Weight",
            [this]
            {
                return m_weight;
            },
            [this](double value)
            {
                m_weight = value;
            }
            ),
        mc_rtc::gui::ArrayInput(
            "Left hand target position robot",
            [this]
            {
                return m_leftHandTargetPositionRobot;
            },
            [this](const Eigen::Vector3d & value)
            {
                m_leftHandTargetPositionRobot = value;
            }
            ),
        mc_rtc::gui::ArrayInput(
            "Right hand target position robot",
            [this]
            {
                return m_rightHandTargetPositionRobot;
            },
            [this](const Eigen::Vector3d & value)
            {
                m_rightHandTargetPositionRobot = value;
            }
            )
        );

    ctl.gui()->addElement(
        {"GMB", "MoveHands"},
        mc_rtc::gui::ComboInput(
            "Robot reference frame",
            ctl.robot().frames(),
            [this]
            {
                return m_robotReferenceFrame;
            },
            [this, &ctl_](const std::string & frame)
            {
                if (frame == m_robotReferenceFrame) return;
                m_robotReferenceFrame = frame;
            }
            ),
        mc_rtc::gui::ComboInput(
            "Left hand frame",
            ctl.robot().frames(),
            [this]
            {
                return m_leftHandFrame;
            },
            [this, &ctl_](const std::string & frame)
            {
                if (frame == m_leftHandFrame) return;
                m_leftHandFrame = frame;
            }
            ),
        mc_rtc::gui::ComboInput(
            "Right hand frame",
            ctl.robot().frames(),
            [this]
            {
                return m_rightHandFrame;
            },
            [this, &ctl_](const std::string & frame)
            {
                if (frame == m_rightHandFrame) return;
                m_rightHandFrame = frame;
            }
            )
        );
}

void MoveHands::removeFromGui(mc_control::fsm::Controller & ctl_)
{
    auto & ctl = static_cast<DemoController&>(ctl_);
    ctl.gui()->removeCategory({"GMB", "MoveHands"});
}

EXPORT_SINGLE_STATE("MoveHands", MoveHands)
