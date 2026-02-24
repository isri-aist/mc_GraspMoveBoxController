#pragma once

#include <mc_control/fsm/State.h>
#include <mc_tasks/TransformTask.h>
#include <memory>

struct GraspLiftBox : mc_control::fsm::State
{
    void configure(const mc_rtc::Configuration & config) override;

    void start(mc_control::fsm::Controller & ctl) override;

    bool run(mc_control::fsm::Controller & ctl) override;

    void teardown(mc_control::fsm::Controller & ctl) override;

private:
    enum class Phase
    {
        Approach,
        Grasping,
        Lift,
        Done
    };

    std::shared_ptr<mc_tasks::TransformTask> m_leftGripperTask, m_rightGripperTask;

    std::string m_objectName;
    std::string m_objectSurfaceLeftGripper, m_objectSurfaceRightGripper;

    double m_stiffness = 1.0;
    double m_weight = 1000.0;
    double m_approachOffset = 0.025;
    double m_liftHeight = 0.10;
    double m_liftPullback = 0.0;
    double m_completionEval = 0.05;
    double m_completionSpeed = 1e-4;

    bool m_contactAdded = false;
    bool m_removeContactAtTeardown = true;
    Phase m_phase = Phase::Approach;
};
