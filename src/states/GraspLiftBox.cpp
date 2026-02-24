#include "GraspLiftBox.h"

#include <console_bridge/console.h>

#include "../DemoController.h"
#include "BaselineWalkingController/CentroidalManager.h"

void GraspLiftBox::configure(const mc_rtc::Configuration &config)
{
    config("objectName", m_objectName);
    config("objectSurfaceLeftGripper", m_objectSurfaceLeftGripper);
    config("objectSurfaceRightGripper", m_objectSurfaceRightGripper);
    config("stiffness", m_stiffness);
    config("weight", m_weight);
    config("approachOffsetZ", m_approachOffset);
    config("liftHeight", m_liftHeight);
    config("liftPullback", m_liftPullback);
    config("completionEval", m_completionEval);
    config("completionSpeed", m_completionSpeed);
    config("removeContactsAtTeardown", m_removeContactAtTeardown);
}

void GraspLiftBox::start(mc_control::fsm::Controller &ctl_)
{
    auto &ctl = static_cast<DemoController &>(ctl_);

    mc_rtc::log::info("Now in approach phase");
    m_phase = Phase::Approach;
    m_contactAdded = false;

    m_leftGripperTask = std::make_shared<mc_tasks::TransformTask>
    (
        ctl.robot().frame("LeftHandWrench"),
        m_stiffness,
        m_weight
    );
    ctl.solver().addTask(m_leftGripperTask);

    auto leftGripperTargetPose = ctl.robot(m_objectName).frame
            (m_objectSurfaceLeftGripper).position();
    leftGripperTargetPose.translation() += ctl.robot().posW().rotation() * Eigen::Vector3d
            (0.0, m_approachOffset, 0.0);
    // [ 0.7  0. 0. -0.7 ] @ [ 0.7 0. 0.7 0. ]
    leftGripperTargetPose.rotation() = Eigen::Quaterniond
            (0.5, 0.5, 0.5, -0.5).toRotationMatrix();
    m_leftGripperTask->target(leftGripperTargetPose);

    m_rightGripperTask = std::make_shared<mc_tasks::TransformTask>
    (
        ctl.robot().frame("RightHandWrench"),
        m_stiffness,
        m_weight
    );
    ctl.solver().addTask(m_rightGripperTask);

    auto rightGripperTargetPose = ctl.robot(m_objectName).frame
            (m_objectSurfaceRightGripper).position();
    rightGripperTargetPose.translation() += ctl.robot().posW().rotation() *
            Eigen::Vector3d(0.0, -m_approachOffset, 0.0);
    // [ 0.7 0. 0. 0.7 ] @ [ 0.7 0. 0.7 0. ]
    rightGripperTargetPose.rotation() = Eigen::Quaterniond
            (0.5, -0.5, 0.5, 0.5).toRotationMatrix();
    m_rightGripperTask->target(rightGripperTargetPose);
}

bool GraspLiftBox::run(mc_control::fsm::Controller &ctl_)
{
    auto &ctl = static_cast<DemoController &>(ctl_);

    const bool completed = (
        m_leftGripperTask->eval().norm() < m_completionEval
        && m_leftGripperTask->speed().norm() < m_completionSpeed
        && m_rightGripperTask->eval().norm() < m_completionEval
        && m_rightGripperTask->speed().norm() < m_completionSpeed
    );

    if (!completed)
    {
        return false;
    }

    if (m_phase == Phase::Approach)
    {
        auto previousLeftTarget = m_leftGripperTask->target();
        previousLeftTarget.translation() = ctl.robot(m_objectName).frame
                (m_objectSurfaceLeftGripper).position().translation();
        m_leftGripperTask->target(previousLeftTarget);

        auto previousRightTarget = m_rightGripperTask->target();
        previousRightTarget.translation() = ctl.robot(m_objectName).frame
                (m_objectSurfaceRightGripper).position().translation();
        m_rightGripperTask->target(previousRightTarget);

        mc_rtc::log::info("Now in grasping phase");
        m_phase = Phase::Grasping;
        return false;
    }

    if (m_phase == Phase::Grasping)
    {
        ctl.addContact
        (
            {
                ctl.robot().name(),
                ctl.robot(m_objectName).name(),
                "LeftHandWrench",
                m_objectSurfaceLeftGripper
            }
        );
        ctl.addContact
        (
            {
                ctl.robot().name(),
                ctl.robot(m_objectName).name(),
                "RightHandWrench",
                m_objectSurfaceRightGripper
            }
        );

        m_contactAdded = true;

        auto leftLiftTarget = m_leftGripperTask->target();
        leftLiftTarget.translation() += Eigen::Vector3d(-m_liftPullback, 0.0, m_liftHeight);
        m_leftGripperTask->target(leftLiftTarget);

        auto rightLiftTarget = m_rightGripperTask->target();
        rightLiftTarget.translation() += Eigen::Vector3d(-m_liftPullback, 0.0, m_liftHeight);
        m_rightGripperTask->target(rightLiftTarget);

        mc_rtc::log::info("Now in lift phase");
        m_phase = Phase::Lift;
        return false;
    }

    if (m_phase == Phase::Lift)
    {
        mc_rtc::log::info("Done");
        m_phase = Phase::Done;
        output("OK");
        return true;
    }

    return false;
}

void GraspLiftBox::teardown(mc_control::fsm::Controller &ctl_)
{
    auto &ctl = static_cast<DemoController &>(ctl_);

    ctl.solver().removeTask(m_leftGripperTask);
    ctl.solver().removeTask(m_rightGripperTask);
    m_leftGripperTask.reset();
    m_rightGripperTask.reset();

    if (m_contactAdded && m_removeContactAtTeardown)
    {
        ctl.removeContact
        (
            {
                ctl.robot().name(),
                ctl.robot(m_objectName).name(),
                "LeftHandWrench",
                m_objectSurfaceLeftGripper
            }
        );
        ctl.removeContact
        (
            {
                ctl.robot().name(),
                ctl.robot(m_objectName).name(),
                "RightHandWrench",
                m_objectSurfaceRightGripper
            }
        );
        m_contactAdded = false;
    }
}

EXPORT_SINGLE_STATE("GraspLiftBox", GraspLiftBox)
