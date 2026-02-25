#pragma once

#include <mc_control/fsm/State.h>
#include <mc_tasks/TransformTask.h>
#include <memory>
#include <vector>

struct GraspMoveBox : mc_control::fsm::State
{
    void configure(const mc_rtc::Configuration &config) override;

    void start(mc_control::fsm::Controller &ctl) override;

    bool run(mc_control::fsm::Controller &ctl) override;

    void teardown(mc_control::fsm::Controller &ctl) override;

private:
    enum class Phase
    {
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

    std::shared_ptr<mc_tasks::TransformTask>
            m_leftGripperTask,
            m_rightGripperTask;

    std::string
            m_objectName,
            m_objectSurfaceLeftGripper,
            m_objectSurfaceRightGripper;

    std::vector<Eigen::Vector3d> m_graspFromPoses, m_dropFromPoses;
    size_t m_graspWaypointIndex = 0, m_dropWaypointIndex = 0;

    double
            m_stiffness = 1.0,
            m_weight = 1000.0,
            m_StartTime = 0.0,
            m_Timeout = 5.0,
            m_approachOffset = 0.0,
            m_liftHeight = 0.1,
            m_liftDistance = 0.0,
            m_dropHeight = 0.1,
            m_dropDistance = 0.0,
            m_completionEval = 0.05,
            m_completionSpeed = 1e-3;

    Eigen::Vector3d
            m_raiseLeftHandPosition = Eigen::Vector3d(0.0, 0.25, 0.0),
            m_raiseRightHandPosition = Eigen::Vector3d(0.0, -0.25, 0.0);

    Eigen::Quaterniond
            m_raiseLeftHandOrientation = Eigen::Quaterniond(0.5, 0.5, 0.5, -0.5),
            m_raiseRightHandOrientation = Eigen::Quaterniond(0.5, -0.5, 0.5, 0.5);

    bool
            m_contactAdded = false,
            m_removeContactAtTeardown = true;

    Phase m_phase = Phase::ApproachBox;

    Eigen::Vector3d relativePose(Eigen::Vector3d absolutePose, sva::PTransformd robotPose);
};
