#pragma once

#include <mc_control/fsm/State.h>
#include <mc_tasks/RelativeEndEffectorTask.h>
#include <mc_tasks/TransformTask.h>

#include "mc_control/Contact.h"

struct GoToBoxPickupPose : mc_control::fsm::State
{
    public:
        void configure(const mc_rtc::Configuration &) override;
        void start(mc_control::fsm::Controller &) override;
        bool run(mc_control::fsm::Controller &) override;
        void teardown(mc_control::fsm::Controller &) override;

    private:
        const std::vector<std::string> LeftArmJoints = {
                "L_SHOULDER_P",
                "L_SHOULDER_R",
                "L_SHOULDER_Y",
                "L_ELBOW_P",
                "L_ELBOW_Y",
                "L_WRIST_R",
                "L_WRIST_Y",
        };
        const std::vector<std::string> RightArmJoints = {
                "R_SHOULDER_P",
                "R_SHOULDER_R",
                "R_SHOULDER_Y",
                "R_ELBOW_P",
                "R_ELBOW_Y",
                "R_WRIST_R",
                "R_WRIST_Y",
        };

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

        std::shared_ptr<mc_tasks::RelativeEndEffectorTask> m_leftGripperTask;
        std::shared_ptr<mc_tasks::RelativeEndEffectorTask> m_rightGripperTask;

        std::string m_objectName;
        std::string m_objectSurfaceLeftGripper;
        std::string m_objectSurfaceRightGripper;

        Phase m_phase = Phase::None;

        double m_stiffness          = 1.0;
        double m_weight             = 1000.0;
        double m_StartTime          = 0.0;
        double m_Timeout            = 5.0;
        double m_approachOffset     = 0.0;
        double m_leftGripperOffset  = 0.0;
        double m_rightGripperOffset = 0.0;
        double m_liftHeight         = 0.1;
        double m_liftDistance       = 0.0;
        double m_dropHeight         = 0.1;
        double m_dropDistance       = 0.0;
        double m_completionEval     = 0.05;
        double m_completionSpeed    = 1e-3;
        double m_BoxHalfWidth       = 0.0;

        bool m_contactAdded            = false;
        bool m_removeContactAtTeardown = true;
        bool m_manualPhaseChange       = false;
        bool m_allowPhaseChange        = true;

        mc_control::Contact m_leftContact{};
        mc_control::Contact m_rightContact{};

        Eigen::Vector3d m_graspFromPoseWorld;
        Eigen::Vector3d m_dropFromPoseWorld;
        Eigen::Vector3d m_Target2DRobot;
        Eigen::Vector3d m_safeLeftHandPositionRobot  = Eigen::Vector3d(0.0, 0.25, 0.0);
        Eigen::Vector3d m_safeRightHandPositionRobot = Eigen::Vector3d(0.0, -0.25, 0.0);

        Eigen::Quaterniond m_leftHandGraspOrientationRobot  = Eigen::Quaterniond(0.5, 0.5, 0.5, -0.5);
        Eigen::Quaterniond m_rightHandGraspOrientationRobot = Eigen::Quaterniond(0.5, -0.5, 0.5, 0.5);

        sva::PTransformd m_leftHandTargetWorld;
        sva::PTransformd m_leftHandTargetRobot;
        sva::PTransformd m_rightHandTargetWorld;
        sva::PTransformd m_rightHandTargetRobot;

        Eigen::Matrix3d  toXYPlane(Eigen::Matrix3d);
        sva::PTransformd toHorizonAlignedPoseWorld(sva::PTransformd, sva::PTransformd);
        Eigen::Vector3d  toPose2DRobot(Eigen::Vector3d, sva::PTransformd);
};
