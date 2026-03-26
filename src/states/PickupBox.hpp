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

        std::string m_robotReferenceFrame;
        std::string m_gripperSurfaceLeftGripper;
        std::string m_gripperSurfaceRightGripper;
        std::string m_objectName;
        std::string m_objectSurfaceLeftGripper;
        std::string m_objectSurfaceRightGripper;

        Phase m_phase;

        double m_stiffness;
        double m_weight;
        double m_admittanceStiffness;
        double m_admittanceDamping;
        double m_leftAdmittanceWrenchTarget;
        double m_rightAdmittanceWrenchTarget;
        double m_admittanceCoefficient;
        double m_completionEval;
        double m_completionSpeed;
        double m_leftGripperContactOffset;
        double m_rightGripperContactOffset;
        double m_boxHalfWidth;
        double m_crouchOffset;

        bool m_removeContactAtTeardown;
        bool m_manualPhaseChange;
        bool m_contactAdded;
        bool m_phaseAdvanceRequested;
        bool m_centroidManagerDidItsJob;

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

        mc_rtc::Configuration m_config;

        void handlePhaseChange(DemoController &);
        void updateStateConfig(DemoController &);
        void addToGui(DemoController &);
        void removeFromGui(DemoController &);
};