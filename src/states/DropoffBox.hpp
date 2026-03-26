#pragma once

#include <mc_control/fsm/State.h>
#include <mc_tasks/TransformTask.h>
#include "../DemoController.h"

struct DropoffBox : mc_control::fsm::State
{
    public:
        void configure(const mc_rtc::Configuration &) override;
        void start(mc_control::fsm::Controller &) override;
        bool run(mc_control::fsm::Controller &) override;
        void teardown(mc_control::fsm::Controller &) override;

    private:
        enum class Phase
        {
            Init,
            LowerBox,
            DropBox,
            Retreat,
            Finished
        };

        std::shared_ptr<mc_tasks::TransformTask> m_leftGripperTask;
        std::shared_ptr<mc_tasks::TransformTask> m_rightGripperTask;

        std::string m_robotReferenceFrame;
        std::string m_objectName;
        std::string m_objectSurfaceLeftGripper;
        std::string m_objectSurfaceRightGripper;
        std::string m_gripperSurfaceLeftGripper;
        std::string m_gripperSurfaceRightGripper;

        Phase m_phase;

        double m_stiffness;
        double m_weight;
        double m_completionEval;
        double m_completionSpeed;
        double m_crouchOffset;
        double m_leftGripperContactOffset;
        double m_rightGripperContactOffset;
        double m_boxHalfWidth;

        bool m_removeContactAtTeardown;
        bool m_manualPhaseChange;
        bool m_contactAdded;
        bool m_phaseAdvanceRequested;
        bool m_centroidManagerDidItsJob;

        Eigen::Vector3d m_leftGraspOffsetBox;
        Eigen::Vector3d m_rightGraspOffsetBox;

        Eigen::Vector3d m_leftApproachOffsetBox;
        Eigen::Vector3d m_rightApproachOffsetBox;

        Eigen::Quaterniond m_leftOrientationBox;
        Eigen::Quaterniond m_rightOrientationBox;

        Eigen::Vector3d m_leftGraspOffsetRobot;
        Eigen::Vector3d m_rightGraspOffsetRobot;

        Eigen::Vector3d m_leftDropPositionRobot;
        Eigen::Vector3d m_rightDropPositionRobot;

        Eigen::Vector3d m_leftApproachOffsetRobot;
        Eigen::Vector3d m_rightApproachOffsetRobot;

        Eigen::Quaterniond m_leftOrientationRobot;
        Eigen::Quaterniond m_rightOrientationRobot;

        mc_rtc::Configuration m_config;

        void handlePhaseChange(DemoController &);
        void updateStateConfig(DemoController &);
        void addToGui(DemoController &);
        void removeFromGui(DemoController &);
};
