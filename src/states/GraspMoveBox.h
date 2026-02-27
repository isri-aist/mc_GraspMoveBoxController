#pragma once

#include <mc_control/fsm/State.h>
#include <mc_tasks/TransformTask.h>
#include <memory>

#include "mc_control/Contact.h"

struct GraspMoveBox: mc_control::fsm::State
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
            WalkToBox,
            RaiseHands,
            ApproachBox,
            GraspBox,
            RaiseBox,
            WalkToDrop,
            LowerBox,
            DropBox,
            RemoveHands
        };

        std::shared_ptr<mc_tasks::TransformTask> m_leftGripperTask,
                                                 m_rightGripperTask;

        std::string m_objectName,
                    m_objectSurfaceLeftGripper,
                    m_objectSurfaceRightGripper;

        Phase m_phase = Phase::None;

        double m_stiffness       = 1.0,
               m_weight          = 1000.0,
               m_StartTime       = 0.0,
               m_Timeout         = 5.0,
               m_approachOffset  = 0.0,
               m_liftHeight      = 0.1,
               m_liftDistance    = 0.0,
               m_dropHeight      = 0.1,
               m_dropDistance    = 0.0,
               m_completionEval  = 0.05,
               m_completionSpeed = 1e-3,
               m_BoxHalfWidth    = 0.0;

        bool m_contactAdded            = false,
             m_removeContactAtTeardown = true,
             m_manualPhaseChange       = false,
             m_allowPhaseChange        = true;

        mc_control::Contact m_leftContact{}, m_rightContact{};

        Eigen::Vector3d m_graspFromPoseWorld,
                        m_dropFromPoseWorld,
                        m_Target2DRobot,
                        m_safeLeftHandPositionRobot  = Eigen::Vector3d(0.0, 0.25, 0.0),
                        m_safeRightHandPositionRobot = Eigen::Vector3d(0.0, -0.25, 0.0);

        Eigen::Quaterniond
                m_leftHandGraspOrientationRobot  = Eigen::Quaterniond(0.5, 0.5, 0.5, -0.5),
                m_rightHandGraspOrientationRobot = Eigen::Quaterniond(0.5, -0.5, 0.5, 0.5);

        sva::PTransformd m_leftHandTargetWorld,
                         m_leftHandTargetRobot,
                         m_rightHandTargetWorld,
                         m_rightHandTargetRobot;

        Eigen::Matrix3d  toXYPlane(Eigen::Matrix3d);
        sva::PTransformd toHorizonAlignedPoseWorld(sva::PTransformd, sva::PTransformd);
        Eigen::Vector3d  toPose2DRobot(Eigen::Vector3d, sva::PTransformd);
};
