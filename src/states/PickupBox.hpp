#pragma once

#include <Eigen/Core>
#include <mc_control/Contact.h>
#include <mc_control/fsm/State.h>
#include <mc_tasks/AdmittanceTask.h>
#include "../DemoController.h"

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
            Init,
            ApproachBox,
            GraspBox,
            RaiseBox,
            Finished
        };

        std::shared_ptr<mc_tasks::force::AdmittanceTask> m_leftGripperTask;
        std::shared_ptr<mc_tasks::force::AdmittanceTask> m_rightGripperTask;

        std::string m_robotReferenceFrame = "CHEST_Y_LINK";
        std::string m_objectName;
        std::string m_objectSurfaceLeftGripper;
        std::string m_objectSurfaceRightGripper;
        std::string m_gripperSurfaceLeftGripper  = "LeftHandSupportPlate";
        std::string m_gripperSurfaceRightGripper = "RightHandSupportPlate";

        Phase m_phase = Phase::Init;

        double m_stiffness                   = 2.0;
        double m_weight                      = 2000.0;
        double m_admittanceStiffness         = 1.0;
        double m_admittanceDamping           = 300.0;
        double m_leftAdmittanceWrenchTarget  = 10.0;
        double m_rightAdmittanceWrenchTarget = 10.0;
        double m_admittanceCoefficient       = 0.001;
        double m_completionEval              = 0.05;
        double m_completionSpeed             = 1e-3;
        double m_boxHalfWidth                = 0.0;
        double m_crouchOffset                = 0.05;
        double m_leftGripperContactOffset    = 0.0;
        double m_rightGripperContactOffset   = 0.0;

        bool m_contactAdded             = false;
        bool m_removeContactAtTeardown  = false;
        bool m_manualPhaseChange        = true;
        bool m_phaseAdvanceRequested    = false;
        bool m_centroidManagerDidItsJob = false;

        mc_control::Contact m_leftContact{};
        mc_control::Contact m_rightContact{};

        sva::ForceVecd m_admittanceCoefficients;

        sva::ForceVecd m_leftCarryWrench;
        sva::ForceVecd m_rightCarryWrench;

        Eigen::VectorXd m_dimStiffness;
        Eigen::VectorXd m_dimDamping;

        Eigen::Vector3d m_leftGraspOffsetBox;
        Eigen::Vector3d m_rightGraspOffsetBox;

        Eigen::Vector3d m_leftApproachOffsetBox;
        Eigen::Vector3d m_rightApproachOffsetBox;

        Eigen::Quaterniond m_leftOrientationBox;
        Eigen::Quaterniond m_rightOrientationBox;

        Eigen::Vector3d m_leftGraspOffsetRobot;
        Eigen::Vector3d m_rightGraspOffsetRobot;

        Eigen::Vector3d m_leftApproachOffsetRobot;
        Eigen::Vector3d m_rightApproachOffsetRobot;

        Eigen::Vector3d m_leftCarryPositionRobot;
        Eigen::Vector3d m_rightCarryPositionRobot;

        Eigen::Quaterniond m_leftOrientationRobot;
        Eigen::Quaterniond m_rightOrientationRobot;

        void handlePhaseChange(DemoController &);
        void updateStateConfig(DemoController &);
        void addToGui(DemoController &);
        void removeFromGui(DemoController &);
};
