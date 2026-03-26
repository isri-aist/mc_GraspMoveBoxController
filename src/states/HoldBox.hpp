#pragma once
#include <mc_control/fsm/State.h>
#include "mc_tasks/RelativeEndEffectorTask.h"

struct HoldBox : mc_control::fsm::State
{
    public:
        void configure(const mc_rtc::Configuration &) override;
        void start(mc_control::fsm::Controller &) override;
        bool run(mc_control::fsm::Controller &) override;
        void teardown(mc_control::fsm::Controller &) override;

    private:
        std::shared_ptr<mc_tasks::RelativeEndEffectorTask> m_leftGripperTask;
        std::shared_ptr<mc_tasks::RelativeEndEffectorTask> m_rightGripperTask;

        std::string m_robotReferenceFrame;
        std::string m_objectName;
        std::string m_objectSurfaceLeftGripper;
        std::string m_objectSurfaceRightGripper;
        std::string m_gripperSurfaceLeftGripper;
        std::string m_gripperSurfaceRightGripper;

        double m_stiffness;
        double m_weight;
        double m_boxHalfWidth;

        Eigen::Vector3d m_leftPositionRobot;
        Eigen::Vector3d m_rightPositionRobot;

        Eigen::Quaterniond m_leftOrientationRobot;
        Eigen::Quaterniond m_rightOrientationRobot;

        mc_rtc::Configuration m_config;

        void addToGui(mc_control::fsm::Controller &);
        void removeFromGui(mc_control::fsm::Controller &);
};