#pragma once

#include <mc_control/fsm/State.h>
#include <mc_tasks/GazeTask.h>


struct LookAtBox : mc_control::fsm::State
{
        bool run(mc_control::fsm::Controller &ctl_) override;
        void start(mc_control::fsm::Controller &ctl_) override;
        void teardown(mc_control::fsm::Controller &ctl_) override;
        void configure(const mc_rtc::Configuration &config) override;

    protected:
        double m_stiffness = 2.0;
        double m_weight    = 500.0;

        std::string m_objectName;
        std::string m_cameraControlFrame;

        Eigen::Vector2d m_error;

        std::shared_ptr<mc_tasks::GazeTask> m_gazeTask;
};
