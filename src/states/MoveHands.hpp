#pragma once

#include <Eigen/Geometry>
#include <mc_control/fsm/State.h>
#include <mc_tasks/RelativeEndEffectorTask.h>

struct MoveHands : mc_control::fsm::State
{
    bool run(mc_control::fsm::Controller & ctl_) override;
    void start(mc_control::fsm::Controller & ctl_) override;
    void teardown(mc_control::fsm::Controller & ctl_) override;
    void configure(const mc_rtc::Configuration & config) override;

    protected:
        std::shared_ptr<mc_tasks::RelativeEndEffectorTask> m_leftGripperTask;
        std::shared_ptr<mc_tasks::RelativeEndEffectorTask> m_rightGripperTask;

        Eigen::Vector3d m_leftHandTargetPositionRobot;
        Eigen::Vector3d m_rightHandTargetPositionRobot;

        Eigen::Quaterniond m_leftHandTargetOrientationRobot;
        Eigen::Quaterniond m_rightHandTargetOrientationRobot;

        std::string m_robotReferenceFrame;
        std::string m_leftHandFrame;
        std::string m_rightHandFrame;

        double m_stiffness;
        double m_weight;

        bool m_started;
        bool m_autoStart;
        bool m_configured = false;

        mc_rtc::Configuration m_config;

        void addToGui(mc_control::fsm::Controller & ctl_);
        void removeFromGui(mc_control::fsm::Controller & ctl_);
};