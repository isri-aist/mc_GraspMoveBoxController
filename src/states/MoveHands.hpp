#pragma once

#include <Eigen/Geometry>
#include <mc_control/fsm/State.h>
#include <mc_tasks/RelativeEndEffectorTask.h>


struct MoveHands : mc_control::fsm::State
{
        bool run(mc_control::fsm::Controller &ctl_) override;
        void start(mc_control::fsm::Controller &ctl_) override;
        void teardown(mc_control::fsm::Controller &ctl_) override;
        void configure(const mc_rtc::Configuration &config) override;

    protected:
        Eigen::Vector3d m_leftHandTargetPositionRobot;
        Eigen::Vector3d m_rightHandTargetPositionRobot;

        Eigen::Quaterniond m_leftHandTargetOrientationRobot  = {0.5, 0.5, 0.5, -0.5};
        Eigen::Quaterniond m_rightHandTargetOrientationRobot = {0.5, -0.5, 0.5, 0.5};

        std::string m_robotReferenceFrame = "CHEST_Y_LINK";
        std::string m_leftHandFrame       = "LeftHandSupportPlate";
        std::string m_rightHandFrame      = "RightHandSupportPlate";

        double m_stiffness = 2.0;
        double m_weight    = 1000.0;

        bool m_started              = false;
        bool m_autoStart            = false;

        std::shared_ptr<mc_tasks::RelativeEndEffectorTask> m_leftGripperTask;
        std::shared_ptr<mc_tasks::RelativeEndEffectorTask> m_rightGripperTask;

        void addToGui(mc_control::fsm::Controller &ctl_);
        void removeFromGui(mc_control::fsm::Controller &ctl_);
};
