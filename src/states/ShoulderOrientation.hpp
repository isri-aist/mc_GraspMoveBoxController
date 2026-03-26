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

        std::string m_leftReferenceFrame;
        std::string m_rightReferenceFrame;

        std::string m_leftShoulderFrame;
        std::string m_rightShoulderFrame;

        std::vector<std::string> m_leftShoulderActiveJoints;
        std::vector<std::string> m_rightShoulderActiveJoints;

        double m_stiffness;
        double m_weight;
        double m_leftShoulderAngle;
        double m_rightShoulderAngle;

        mc_rtc::Configuration m_config;

        void        updateOrientationTask(mc_control::fsm::Controller &) const;
        void        addToGui(mc_control::fsm::Controller &);
        static void removeFromGui(mc_control::fsm::Controller &);
};
