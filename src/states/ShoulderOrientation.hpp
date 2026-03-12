#pragma once

#include <Eigen/Core>
#include <mc_control/fsm/State.h>
#include <mc_tasks/OrientationTask.h>

struct ShoulderOrientation : mc_control::fsm::State
{
    public:
        void configure(const mc_rtc::Configuration &) override;
        void start(mc_control::fsm::Controller &) override;
        bool run(mc_control::fsm::Controller &) override;
        void teardown(mc_control::fsm::Controller &) override;

    private:
        std::shared_ptr<mc_tasks::OrientationTask> m_leftElbowOrientationTask;
        std::shared_ptr<mc_tasks::OrientationTask> m_rightElbowOrientationTask;

        std::string m_leftReferenceFrame  = "L_SHOULDER_R_LINK";
        std::string m_rightReferenceFrame = "R_SHOULDER_R_LINK";

        std::string m_leftShoulderFrame  = "L_SHOULDER_Y_LINK";
        std::string m_rightShoulderFrame = "R_SHOULDER_Y_LINK";

        std::vector<std::string> m_leftShoulderActiveJoints  = {"L_SHOULDER_R"};
        std::vector<std::string> m_rightShoulderActiveJoints = {"R_SHOULDER_R"};

        double m_stiffness          = 2.0;
        double m_weight             = 2000.0;
        double m_leftShoulderAngle  = 5.0 * M_PI / 180.0;
        double m_rightShoulderAngle = -5.0 * M_PI / 180.0;

        void setOrientationTaskGoal(mc_control::fsm::Controller &) const;
};
