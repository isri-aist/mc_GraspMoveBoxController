#pragma once

#include <mc_control/fsm/State.h>
#include <mc_tasks/OrientationTask.h>
#include <mc_tasks/TransformTask.h>

#include "mc_control/Contact.h"

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
            None,
            LowerBox,
            DropBox
        };

        std::shared_ptr<mc_tasks::TransformTask>   m_leftGripperTask;
        std::shared_ptr<mc_tasks::TransformTask>   m_rightGripperTask;
        std::shared_ptr<mc_tasks::OrientationTask> m_leftElbowOrientationTask;
        std::shared_ptr<mc_tasks::OrientationTask> m_rightElbowOrientationTask;

        std::string m_robotReferenceFrame = "CHEST_Y_LINK";
        std::string m_objectName;
        std::string m_objectSurfaceLeftGripper;
        std::string m_objectSurfaceRightGripper;
        std::string m_gripperSurfaceLeftGripper  = "LeftHandSupportPlate";
        std::string m_gripperSurfaceRightGripper = "RightHandSupportPlate";

        Phase m_phase = Phase::None;

        double m_stiffness                 = 2.0;
        double m_weight                    = 2000.0;
        double m_startTime                 = 0.0;
        double m_timeout                   = 5.0;
        double m_completionEval            = 0.05;
        double m_completionSpeed           = 1e-3;
        double m_boxHalfWidth              = 0.0;
        double m_leftShoulderZAngle        = -5.0 * M_PI / 180.0;
        double m_rightShoulderZAngle       = 5.0 * M_PI / 180.0;
        double m_crouchOffset              = 0.05;
        double m_leftGripperContactOffset  = 0.0;
        double m_rightGripperContactOffset = 0.0;
        double m_approachOffset            = 0.0;
        double m_refComZ                   = 0.0;

        bool m_contactAdded            = false;
        bool m_removeContactAtTeardown = true;
        bool m_manualPhaseChange       = true;
        bool m_allowPhaseChange        = true;

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

        Eigen::Quaterniond m_leftOrientationRobot;
        Eigen::Quaterniond m_rightOrientationRobot;
};
