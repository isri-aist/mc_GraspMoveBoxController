#pragma once

#include <Eigen/Core>
#include <mc_control/fsm/State.h>
#include <mc_tasks/OrientationTask.h>
#include <mc_tasks/TransformTask.h>

#include "mc_control/Contact.h"

struct PickupBox : mc_control::fsm::State
{
    public:
        void configure(const mc_rtc::Configuration &) override;
        void start(mc_control::fsm::Controller &) override;
        bool run(mc_control::fsm::Controller &) override;
        void teardown(mc_control::fsm::Controller &) override;

    private:
        enum Phase
        {
            None,
            RaiseHands,
            ApproachBox,
            GraspBox,
            RaiseBox
        };

        std::shared_ptr<mc_tasks::OrientationTask> m_leftElbowOrientationTask;
        std::shared_ptr<mc_tasks::OrientationTask> m_rightElbowOrientationTask;
        std::shared_ptr<mc_tasks::TransformTask>   m_leftGripperTask;
        std::shared_ptr<mc_tasks::TransformTask>   m_rightGripperTask;

        std::string m_robotReferenceFrame = "CHEST_Y_LINK";
        std::string m_objectName;
        std::string m_objectSurfaceLeftGripper;
        std::string m_objectSurfaceRightGripper;

        Phase m_phase = Phase::None;

        double m_stiffness       = 2.0;
        double m_weight          = 2000.0;
        double m_StartTime       = 0.0;
        double m_Timeout         = 5.0;
        double m_completionEval  = 0.05;
        double m_completionSpeed = 1e-3;
        double m_BoxHalfWidth    = 0.0;

        bool m_contactAdded            = false;
        bool m_removeContactAtTeardown = false;
        bool m_manualPhaseChange       = false;
        bool m_allowPhaseChange        = true;

        mc_control::Contact m_leftContact{};
        mc_control::Contact m_rightContact{};

        Eigen::Vector3d m_leftApproachOffset;
        Eigen::Vector3d m_rightApproachOffset;

        Eigen::Vector3d m_leftGraspOffset;
        Eigen::Vector3d m_rightGraspOffset;

        Eigen::Vector3d m_leftCarryPositionRobot;
        Eigen::Vector3d m_rightCarryPositionRobot;

        Eigen::Quaterniond m_leftCarryOrientationRobot;
        Eigen::Quaterniond m_rightCarryOrientationRobot;

        Eigen::Vector3d m_leftRaisePositionRobot  = {0.0, 0.25, 0.0};
        Eigen::Vector3d m_rightRaisePositionRobot = {0.0, -0.25, 0.0};

        Eigen::Quaterniond m_leftRaiseOrientationRobot  = {0.5, 0.5, 0.5, -0.5};
        Eigen::Quaterniond m_rightRaiseOrientationRobot = {0.5, -0.5, 0.5, 0.5};
};
