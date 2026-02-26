#include "GraspMoveBox.h"

#include <console_bridge/console.h>
#include <Eigen/Geometry>
#include <mc_rbdyn/rpy_utils.h>
#include <cmath>

#include "../DemoController.h"
#include "BaselineWalkingController/CentroidalManager.h"
#include "BaselineWalkingController/FootManager.h"

void GraspMoveBox::configure(const mc_rtc::Configuration &config)
{
    config("objectName", m_objectName);
    config("objectSurfaceLeftGripper", m_objectSurfaceLeftGripper);
    config("objectSurfaceRightGripper", m_objectSurfaceRightGripper);
    config("graspFromPose", m_graspFromPose);
    config("dropFromPose", m_dropFromPose);
    config("stiffness", m_stiffness);
    config("weight", m_weight);
    config("approachOffset", m_approachOffset);
    config("liftHeight", m_liftHeight);
    config("liftDistance", m_liftDistance);
    config("dropHeight", m_dropHeight);
    config("dropDistance", m_dropDistance);
    config("completionEval", m_completionEval);
    config("completionSpeed", m_completionSpeed);
    config("removeContactsAtTeardown", m_removeContactAtTeardown);
    config("raiseLeftHandPosition", m_raiseLeftHandPosition);
    config("raiseRightHandPosition", m_raiseRightHandPosition);
    config("raiseLeftHandOrientation", m_raiseLeftHandOrientation);
    config("raiseRightHandOrientation", m_raiseRightHandOrientation);
    config("Timeout", m_Timeout);

    m_raiseLeftHandPose = {
                    m_raiseLeftHandOrientation.toRotationMatrix(),
                    m_raiseLeftHandPosition
            };
    m_raiseRightHandPose = {
                    m_raiseRightHandOrientation.toRotationMatrix(),
                    m_raiseRightHandPosition
            };
    m_contactAdded = false;
}

void GraspMoveBox::start(mc_control::fsm::Controller &ctl_)
{
    auto &ctl = static_cast<DemoController &>(ctl_);

    m_leftGripperTask =
            std::make_shared<mc_tasks::TransformTask>(
                                                      ctl.robot().frame("LeftHandWrench"),
                                                      m_stiffness,
                                                      m_weight
                                                     );
    m_rightGripperTask =
            std::make_shared<mc_tasks::TransformTask>(
                                                      ctl.robot().frame("RightHandWrench"),
                                                      m_stiffness,
                                                      m_weight
                                                     );

    m_leftContact = mc_control::Contact(
                                        ctl.robot().name(),
                                        ctl.robot(m_objectName).name(),
                                        "RightHandWrench",
                                        m_objectSurfaceRightGripper,
                                        mc_rbdyn::Contact::defaultFriction,
                                        Eigen::Vector6d::Ones()
                                       );
    m_rightContact = mc_control::Contact(
                                         ctl.robot().name(),
                                         ctl.robot(m_objectName).name(),
                                         "RightHandWrench",
                                         m_objectSurfaceRightGripper,
                                         mc_rbdyn::Contact::defaultFriction,
                                         Eigen::Vector6d::Ones()
                                        );

    m_BoxHalfWidth = 0.5 * (
                         ctl.robot(m_objectName)
                            .frame(m_objectSurfaceLeftGripper)
                            .position()
                            .translation()
                         - ctl.robot(m_objectName)
                              .frame(m_objectSurfaceRightGripper)
                              .position()
                              .translation()
                     ).norm();
}

bool GraspMoveBox::run(mc_control::fsm::Controller &ctl_)
{
    auto &ctl = static_cast<DemoController &>(ctl_);

    // This is a hack to ensure the object is visible in mc_mujoco because for some reason the
    // box position does not change in the visualization
    if (m_contactAdded)
    {
        const auto setPosWCall = m_objectName + "::SetPosW";
        if (ctl.datastore().has(setPosWCall))
        {
            const auto &objectPosW = ctl.robot(m_objectName).posW();
            ctl.datastore().call<void, const sva::PTransformd &>(setPosWCall, objectPosW);
        }
    }

    if (m_phase == Phase::None)
    {
        m_TargetRelative2D = relativePose2D(m_graspFromPose, ctl.robot().posW());
        ctl.footManager_->reset();
        ctl.footManager_->walkToRelativePose(m_TargetRelative2D);

        mc_rtc::log::info("Now in walk to box phase");
        m_phase = Phase::WalkToBox;
    }

    if (m_phase == Phase::WalkToBox && !ctl.footManager_->footstepQueue().empty()) return false;

    if (m_phase == Phase::WalkToBox && ctl.footManager_->footstepQueue().empty())
    {
        mc_rtc::log::info("Now in raise hands phase");
        m_phase = Phase::RaiseHands;

        m_StartTime = ctl.t();

        auto &robotBodyFrame = ctl.robot().frame("BODY");

        m_leftHandTarget.translation() = m_raiseLeftHandPosition;
        m_leftHandTarget.rotation()    = m_raiseLeftHandOrientation.toRotationMatrix();
        m_leftGripperTask->target(robotBodyFrame, m_leftHandTarget);
        ctl.solver().addTask(m_leftGripperTask);

        sva::PTransformd rightTarget;
        rightTarget.translation() = m_raiseRightHandPosition;
        rightTarget.rotation()    = m_raiseRightHandOrientation.toRotationMatrix();
        m_rightGripperTask->target(robotBodyFrame, rightTarget);
        ctl.solver().addTask(m_rightGripperTask);
        return false;
    }

    bool completed = (
        m_leftGripperTask->eval().norm() < m_completionEval
        && m_leftGripperTask->speed().norm() < m_completionSpeed
        && m_rightGripperTask->eval().norm() < m_completionEval
        && m_rightGripperTask->speed().norm() < m_completionSpeed
    );

    if (m_StartTime + m_Timeout < ctl.t()) completed = true;

    if (m_phase == Phase::RaiseHands && completed)
    {
        // set to max double to deactivate the timeout
        m_StartTime = std::numeric_limits<double>::max();

        mc_rtc::log::info("Now in approach phase");
        m_phase = Phase::ApproachBox;

        auto leftGripperTargetPose = ctl.robot(m_objectName).frame(m_objectSurfaceLeftGripper).
                                         position();
        leftGripperTargetPose.translation() += ctl.robot().posW().rotation() * Eigen::Vector3d(
             0.0,
             m_approachOffset,
             0.0
            );
        leftGripperTargetPose.rotation() = m_raiseLeftHandOrientation.toRotationMatrix();
        m_leftGripperTask->target(leftGripperTargetPose);

        auto rightGripperTargetPose = ctl.robot(m_objectName).frame(m_objectSurfaceRightGripper).
                                          position();
        rightGripperTargetPose.translation() += ctl.robot().posW().rotation() *
                Eigen::Vector3d(0.0, -m_approachOffset, 0.0);
        rightGripperTargetPose.rotation() = m_raiseRightHandOrientation.toRotationMatrix();
        m_rightGripperTask->target(rightGripperTargetPose);
        return false;
    }

    if (m_phase == Phase::ApproachBox && completed)
    {
        mc_rtc::log::info("Now in grasping phase");
        m_phase = Phase::GraspBox;

        auto previousLeftTarget          = m_leftGripperTask->target();
        previousLeftTarget.translation() = ctl.robot(m_objectName)
                                              .frame(m_objectSurfaceLeftGripper)
                                              .position()
                                              .translation();
        m_leftGripperTask->target(previousLeftTarget);

        auto previousRightTarget          = m_rightGripperTask->target();
        previousRightTarget.translation() = ctl.robot(m_objectName)
                                               .frame(m_objectSurfaceRightGripper)
                                               .position()
                                               .translation();
        m_rightGripperTask->target(previousRightTarget);

        return false;
    }

    if (m_phase == Phase::GraspBox && completed)
    {
        mc_rtc::log::info("Now in lift phase");
        m_phase = Phase::RaiseBox;

        ctl.addContact(m_leftContact);
        ctl.addContact(m_rightContact);

        m_contactAdded = true;

        return false;
    }

    if (m_phase == Phase::RaiseBox || m_phase == Phase::WalkToDrop || m_phase == Phase::DropBox)
    {
        auto &robotBodyFrame = ctl.robot().frame("BODY");

        sva::PTransformd leftLiftTarget = m_leftGripperTask->target();
        leftLiftTarget.translation()    = Eigen::Vector3d(
                                                       m_liftDistance,
                                                       m_BoxHalfWidth,
                                                       m_liftHeight
                                                      );
        leftLiftTarget.rotation() = m_raiseLeftHandOrientation.toRotationMatrix();
        m_leftGripperTask->target(robotBodyFrame, leftLiftTarget);

        sva::PTransformd rightLiftTarget = m_rightGripperTask->target();
        rightLiftTarget.translation()    = Eigen::Vector3d(
                                                        m_liftDistance,
                                                        -m_BoxHalfWidth,
                                                        m_liftHeight
                                                       );
        rightLiftTarget.rotation() = m_raiseRightHandOrientation.toRotationMatrix();
        m_rightGripperTask->target(robotBodyFrame, rightLiftTarget);
    }

    if (m_phase == Phase::RaiseBox && completed)
    {
        mc_rtc::log::info("Now in walk to drop phase");
        m_phase                            = Phase::WalkToDrop;
        Eigen::Vector3d targetRelativePose = relativePose2D(m_dropFromPose, ctl.robot().posW());

        mc_rtc::log::info(
                          "target: {}, robot {}, relative target: {}",
                          m_dropFromPose.transpose(),
                          ctl.robot().posW().translation().transpose(),
                          targetRelativePose.transpose()
                         );

        ctl.footManager_->reset();
        ctl.footManager_->walkToRelativePose(targetRelativePose);

        return false;
    }

    if (m_phase == Phase::WalkToDrop && ctl.footManager_->footstepQueue().empty())
    {
        mc_rtc::log::info("Now in lower box phase");
        m_phase = Phase::LowerBox;

        auto &robotBodyFrame = ctl.robot().frame("BODY");

        sva::PTransformd leftDropTarget = m_leftGripperTask->target();
        leftDropTarget.translation()    = Eigen::Vector3d(
                                                       m_dropDistance,
                                                       m_BoxHalfWidth,
                                                       m_dropHeight
                                                      );
        leftDropTarget.rotation() = m_raiseLeftHandOrientation.toRotationMatrix();

        m_leftGripperTask->target(robotBodyFrame, leftDropTarget);

        sva::PTransformd rightDropTarget = m_rightGripperTask->target();
        rightDropTarget.translation()    = Eigen::Vector3d(
                                                        m_dropDistance,
                                                        -m_BoxHalfWidth,
                                                        m_dropHeight
                                                       );
        rightDropTarget.rotation() = m_raiseRightHandOrientation.toRotationMatrix();
        m_rightGripperTask->target(robotBodyFrame, rightDropTarget);
    }

    if (m_phase == Phase::LowerBox && completed)
    {
        mc_rtc::log::info("Now in drop box phase");
        m_phase = Phase::DropBox;

        if (m_contactAdded)
        {
            ctl.removeContact(m_leftContact);
            ctl.removeContact(m_leftContact);
            m_contactAdded = false;
        }

        auto leftGripperTarget = ctl.robot(m_objectName).frame(m_objectSurfaceLeftGripper).
                                     position();
        leftGripperTarget.translation() += ctl.robot().posW().rotation() * Eigen::Vector3d(
             0.0,
             m_approachOffset,
             0.0
            );
        leftGripperTarget.rotation() = m_raiseLeftHandOrientation.toRotationMatrix();
        m_leftGripperTask->target(leftGripperTarget);

        auto rightGripperTarget = ctl.robot(m_objectName).frame(m_objectSurfaceRightGripper).
                                      position();
        rightGripperTarget.translation() += ctl.robot().posW().rotation() * Eigen::Vector3d(
             0.0,
             -m_approachOffset,
             0.0
            );
        rightGripperTarget.rotation() = m_raiseRightHandOrientation.toRotationMatrix();
        m_rightGripperTask->target(rightGripperTarget);
    }

    if (m_phase == Phase::DropBox && completed)
    {
        mc_rtc::log::info("Now in remove hands phase");
        m_phase = Phase::RemoveHands;

        m_StartTime          = ctl.t();
        auto &robotBodyFrame = ctl.robot().frame("BODY");

        sva::PTransformd leftTarget;
        leftTarget.translation() = m_raiseLeftHandPosition;
        m_leftGripperTask->target(robotBodyFrame, leftTarget);
        ctl.solver().addTask(m_leftGripperTask);

        sva::PTransformd rightTarget;
        rightTarget.translation() = m_raiseRightHandPosition;
        m_rightGripperTask->target(robotBodyFrame, rightTarget);
        ctl.solver().addTask(m_rightGripperTask);
        return false;
    }

    if (m_phase == Phase::RemoveHands && completed)
    {
        output("OK");
        return true;
    }

    return false;
}

void GraspMoveBox::teardown(mc_control::fsm::Controller &ctl_)
{
    auto &ctl = static_cast<DemoController &>(ctl_);

    ctl.solver().removeTask(m_leftGripperTask);
    ctl.solver().removeTask(m_rightGripperTask);
    m_leftGripperTask.reset();
    m_rightGripperTask.reset();

    if (m_contactAdded && m_removeContactAtTeardown)
    {
        ctl.removeContact(m_leftContact);
        ctl.removeContact(m_leftContact);
        m_contactAdded = false;
    }
}

sva::PTransformd GraspMoveBox::horizonAlignedRelativePose(
        Eigen::Quaterniond absoluteOrientation,
        Eigen::Vector3d    absolutePosition,
        sva::PTransformd   robotPose
        )
{
    return horizonAlignedRelativePose({absoluteOrientation, absolutePosition}, robotPose);
}

sva::PTransformd GraspMoveBox::horizonAlignedRelativePose(
        sva::PTransformd absolutePose,
        sva::PTransformd robotPose
        )
{
    double           angle(mc_rbdyn::rpyFromMat(robotPose.rotation()).z());
    sva::PTransformd relativePose(sva::RotZ(-angle), Eigen::Vector3d::Zero());

    relativePose.translation() = relativePose.rotation() * (
                                     absolutePose.translation() - robotPose.translation());

    return relativePose;
}

Eigen::Vector3d GraspMoveBox::relativePose2D(
        Eigen::Vector3d  absolutePose,
        sva::PTransformd robotPose
        )
{
    double          angle(mc_rbdyn::rpyFromMat(robotPose.rotation()).z());
    Eigen::Matrix2d rotation = Eigen::Rotation2Dd(-angle).toRotationMatrix();

    Eigen::Vector2d relativePosition(
                                     absolutePose.x() - robotPose.translation().x(),
                                     absolutePose.y() - robotPose.translation().y()
                                    );
    relativePosition = rotation.inverse() * relativePosition;

    Eigen::Vector3d relativePose(
                                 relativePosition.x(),
                                 relativePosition.y(),
                                 absolutePose.z() - angle
                                );

    return relativePose;
}

EXPORT_SINGLE_STATE("GraspMoveBox", GraspMoveBox)
