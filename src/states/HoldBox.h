#pragma once
#include <mc_control/fsm/State.h>
#include <mc_tasks/RelativeEndEffectorTask.h>

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

        double m_stiffness = 2.0;
        double m_weight    = 2000.0;

        std::string m_robotReferenceFrame = "CHEST_Y_LINK";
        std::string m_objectName;
        std::string m_objectSurfaceLeftGripper;
        std::string m_objectSurfaceRightGripper;

        Eigen::Vector3d m_leftPositionRobot;
        Eigen::Vector3d m_rightPositionRobot;

        Eigen::Quaterniond m_leftOrientationRobot;
        Eigen::Quaterniond m_rightOrientationRobot;
};
