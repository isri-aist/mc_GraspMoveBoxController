#include "GraspMoveBox.h"

#include <Eigen/Geometry>
#include <mc_rtc/gui/Label.h>
#include <mc_tasks/RelativeEndEffectorTask.h>

#include "../DemoController.h"
#include "BaselineWalkingController/CentroidalManager.h"
#include "BaselineWalkingController/FootManager.h"


void GraspMoveBox::configure(const mc_rtc::Configuration &config)
{
    mc_rtc::log::info("\n{}", config.dump(true, true));

    config("objectName", m_objectName);
    config("objectSurfaceLeftGripper", m_objectSurfaceLeftGripper);
    config("objectSurfaceRightGripper", m_objectSurfaceRightGripper);
    config("graspFromPoseWorld", m_graspFromPoseWorld);
    config("dropFromPoseWorld", m_dropFromPoseWorld);
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
    config("safeLeftHandPositionRobot", m_safeLeftHandPositionRobot);
    config("safeRightHandPositionRobot", m_safeRightHandPositionRobot);
    config("LeftHandGraspOrientationRobot", m_leftHandGraspOrientationRobot);
    config("RightHandGraspOrientationRobot", m_rightHandGraspOrientationRobot);
    config("manualPhaseChange", m_manualPhaseChange);
    config("Timeout", m_Timeout);

    m_contactAdded     = false;
    m_allowPhaseChange = !m_manualPhaseChange;
}

void GraspMoveBox::start(mc_control::fsm::Controller &ctl_)
{
    auto &ctl = static_cast<DemoController &>(ctl_);

    // for (const auto &joint : ctl.robot().mb().joints()) mc_rtc::log::info("{}", joint.name());

    ctl.gui()->addElement({"GraspMoveBox"}, mc_rtc::gui::Button("Next Phase", [this] { m_allowPhaseChange = true; }));

    m_leftGripperTask = std::make_shared<mc_tasks::RelativeEndEffectorTask>(
            "LeftHandWrench", ctl.robots(), 0, "CHEST_Y_LINK", m_stiffness, m_weight);
    m_leftGripperTask->selectActiveJoints(ctl.solver(), LeftArmJoints);
    ctl.gui()->addElement(
            {"GraspMoveBox"},
            mc_rtc::gui::Label(
                    "Left gripper distance to box and speed",
                    [this]
                    {
                        std::string data = std::to_string(m_leftGripperTask->eval().norm());
                        data += "\t";
                        data += std::to_string(m_leftGripperTask->speed().norm());
                        return data;
                    }));


    m_rightGripperTask = std::make_shared<mc_tasks::RelativeEndEffectorTask>(
            "RightHandWrench", ctl.robots(), 0, "CHEST_Y_LINK", m_stiffness, m_weight);
    m_rightGripperTask->selectActiveJoints(ctl.solver(), RightArmJoints);
    ctl.gui()->addElement(
            {"GraspMoveBox"},
            mc_rtc::gui::Label(
                    "Right gripper distance to box and speed",
                    [this]
                    {
                        std::string data = std::to_string(m_rightGripperTask->eval().norm());
                        data += "\t";
                        data += std::to_string(m_rightGripperTask->speed().norm());
                        return data;
                    }));

    m_leftContact = mc_control::Contact(
            ctl.robot().name(),
            ctl.robot(m_objectName).name(),
            "LeftHandWrench",
            m_objectSurfaceLeftGripper,
            mc_rbdyn::Contact::defaultFriction,
            Eigen::Vector6d::Ones());

    m_rightContact = mc_control::Contact(
            ctl.robot().name(),
            ctl.robot(m_objectName).name(),
            "RightHandWrench",
            m_objectSurfaceRightGripper,
            mc_rbdyn::Contact::defaultFriction,
            Eigen::Vector6d::Ones());

    m_BoxHalfWidth = 0.5 *
            (ctl.robot(m_objectName).frame(m_objectSurfaceLeftGripper).position().translation() -
             ctl.robot(m_objectName).frame(m_objectSurfaceRightGripper).position().translation())
                    .norm();
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
        if (!m_allowPhaseChange) return false;
        if (m_manualPhaseChange) m_allowPhaseChange = false;

        mc_rtc::log::info("Now in walk to box phase");
        m_phase = Phase::WalkToBox;

        m_Target2DRobot = toPose2DRobot(m_graspFromPoseWorld, ctl.robot().posW());
        ctl.footManager_->reset();
        ctl.footManager_->walkToRelativePose(m_Target2DRobot);

        return false;
    }

    if (m_phase == Phase::WalkToBox && !ctl.footManager_->footstepQueue().empty()) return false;

    if (m_phase == Phase::WalkToBox && ctl.footManager_->footstepQueue().empty())
    {
        if (!m_allowPhaseChange) return false;
        if (m_manualPhaseChange) m_allowPhaseChange = false;

        mc_rtc::log::info("Now in raise hands phase");
        m_phase = Phase::RaiseHands;

        m_StartTime = ctl.t();

        m_leftHandTargetRobot.translation() = m_safeLeftHandPositionRobot;
        m_leftHandTargetRobot.rotation()    = m_leftHandGraspOrientationRobot.toRotationMatrix();
        m_leftGripperTask->set_ef_pose(m_leftHandTargetRobot);
        ctl.solver().addTask(m_leftGripperTask);

        m_rightHandTargetRobot.translation() = m_safeRightHandPositionRobot;
        m_rightHandTargetRobot.rotation()    = m_rightHandGraspOrientationRobot.toRotationMatrix();
        m_rightGripperTask->set_ef_pose(m_rightHandTargetRobot);
        ctl.solver().addTask(m_rightGripperTask);

        return false;
    }

    bool completed =
            (m_leftGripperTask->eval().norm() < m_completionEval &&
             m_leftGripperTask->speed().norm() < m_completionSpeed &&
             m_rightGripperTask->eval().norm() < m_completionEval &&
             m_rightGripperTask->speed().norm() < m_completionSpeed);

    if (m_StartTime + m_Timeout < ctl.t())
    {
        mc_rtc::log::info("raise hands timed out");
        completed = true;
    }

    if (m_phase == Phase::RaiseHands && completed)
    {
        if (!m_allowPhaseChange) return false;
        if (m_manualPhaseChange) m_allowPhaseChange = false;

        mc_rtc::log::info("Now in approach phase");
        m_phase = Phase::ApproachBox;

        // set to max double to deactivate the timeout
        m_StartTime = std::numeric_limits<double>::max();

        m_leftHandTargetWorld = ctl.robot(m_objectName).frame(m_objectSurfaceLeftGripper).position();
        m_leftHandTargetRobot.translation() =
                m_leftHandTargetWorld.translation() - ctl.robot().frame("CHEST_Y_LINK").position().translation();
        m_leftHandTargetRobot.translation() += Eigen::Vector3d(0.0, m_approachOffset, 0.0);
        m_leftHandTargetRobot.rotation() = m_leftHandGraspOrientationRobot.toRotationMatrix();
        m_leftGripperTask->set_ef_pose(m_leftHandTargetRobot);

        m_rightHandTargetWorld = ctl.robot(m_objectName).frame(m_objectSurfaceRightGripper).position();
        m_rightHandTargetRobot.translation() =
                m_rightHandTargetWorld.translation() - ctl.robot().frame("CHEST_Y_LINK").position().translation();
        m_rightHandTargetRobot.translation() += Eigen::Vector3d(0.0, -m_approachOffset, 0.0);
        m_rightHandTargetRobot.rotation() = m_rightHandGraspOrientationRobot.toRotationMatrix();
        m_rightGripperTask->set_ef_pose(m_rightHandTargetRobot);

        return false;
    }

    if (m_phase == Phase::ApproachBox && completed)
    {
        if (!m_allowPhaseChange) return false;
        if (m_manualPhaseChange) m_allowPhaseChange = false;

        mc_rtc::log::info("Now in grasping phase");
        m_phase = Phase::GraspBox;

        m_leftHandTargetWorld            = ctl.robot(m_objectName).frame(m_objectSurfaceLeftGripper).position();
        m_leftHandTargetRobot            = ctl.robot().frame("CHEST_Y_LINK").position() * m_leftHandTargetWorld;
        m_leftHandTargetRobot.rotation() = m_leftHandGraspOrientationRobot.toRotationMatrix();
        m_leftGripperTask->set_ef_pose(m_leftHandTargetRobot);

        m_rightHandTargetWorld            = ctl.robot(m_objectName).frame(m_objectSurfaceRightGripper).position();
        m_rightHandTargetRobot            = ctl.robot().frame("CHEST_Y_LINK").position() * m_rightHandTargetWorld;
        m_rightHandTargetRobot.rotation() = m_rightHandGraspOrientationRobot.toRotationMatrix();
        m_rightGripperTask->set_ef_pose(m_rightHandTargetRobot);

        return false;
    }

    if (m_phase == Phase::GraspBox && completed)
    {
        if (!m_allowPhaseChange) return false;
        if (m_manualPhaseChange) m_allowPhaseChange = false;

        mc_rtc::log::info("Now in lift phase");
        m_phase = Phase::RaiseBox;

        ctl.addContact(m_leftContact);
        ctl.addContact(m_rightContact);

        m_leftHandTargetRobot.translation() = Eigen::Vector3d(m_liftDistance, m_BoxHalfWidth, m_liftHeight);
        m_leftHandTargetRobot.rotation()    = m_leftHandGraspOrientationRobot.toRotationMatrix();
        m_leftGripperTask->set_ef_pose(m_leftHandTargetRobot);

        m_rightHandTargetRobot.translation() = Eigen::Vector3d(m_liftDistance, -m_BoxHalfWidth, m_liftHeight);
        m_rightHandTargetRobot.rotation()    = m_rightHandGraspOrientationRobot.toRotationMatrix();
        m_rightGripperTask->set_ef_pose(m_rightHandTargetRobot);

        m_contactAdded = true;

        return false;
    }

    if (m_phase == Phase::RaiseBox && completed)
    {
        if (!m_allowPhaseChange) return false;
        if (m_manualPhaseChange) m_allowPhaseChange = false;

        mc_rtc::log::info("Now in walk to drop phase");
        m_phase = Phase::WalkToDrop;

        Eigen::Vector3d targetRelativePose = toPose2DRobot(m_dropFromPoseWorld, ctl.robot().posW());

        ctl.footManager_->reset();
        ctl.footManager_->walkToRelativePose(targetRelativePose);

        return false;
    }

    if (m_phase == Phase::WalkToDrop && ctl.footManager_->footstepQueue().empty())
    {
        if (!m_allowPhaseChange) return false;
        if (m_manualPhaseChange) m_allowPhaseChange = false;

        mc_rtc::log::info("Now in lower box phase");
        m_phase = Phase::LowerBox;

        m_leftHandTargetRobot            = Eigen::Vector3d(m_dropDistance, m_BoxHalfWidth, m_dropHeight);
        m_leftHandTargetRobot.rotation() = m_leftHandGraspOrientationRobot.toRotationMatrix();
        m_leftGripperTask->set_ef_pose(m_leftHandTargetRobot);

        m_rightHandTargetRobot.translation() = Eigen::Vector3d(m_dropDistance, -m_BoxHalfWidth, m_dropHeight);
        m_rightHandTargetRobot.rotation()    = m_rightHandGraspOrientationRobot.toRotationMatrix();
        m_rightGripperTask->set_ef_pose(m_rightHandTargetRobot);
    }

    if (m_phase == Phase::LowerBox && completed)
    {
        if (!m_allowPhaseChange) return false;
        if (m_manualPhaseChange) m_allowPhaseChange = false;

        mc_rtc::log::info("Now in drop box phase");
        m_phase = Phase::DropBox;

        if (m_contactAdded)
        {
            ctl.removeContact(m_leftContact);
            ctl.removeContact(m_leftContact);
            m_contactAdded = false;
        }

        m_leftHandTargetRobot = Eigen::Vector3d(m_dropDistance, m_BoxHalfWidth + m_approachOffset, m_dropHeight);
        m_leftHandTargetRobot.rotation() = m_leftHandGraspOrientationRobot.toRotationMatrix();
        m_leftGripperTask->set_ef_pose(m_leftHandTargetRobot);

        m_rightHandTargetRobot = Eigen::Vector3d(m_dropDistance, -m_BoxHalfWidth - m_approachOffset, m_dropHeight);
        m_rightHandTargetRobot.rotation() = m_rightHandGraspOrientationRobot.toRotationMatrix();
        m_rightGripperTask->set_ef_pose(m_rightHandTargetRobot);
    }

    if (m_phase == Phase::DropBox && completed)
    {
        if (!m_allowPhaseChange) return false;
        if (m_manualPhaseChange) m_allowPhaseChange = false;

        mc_rtc::log::info("Now in remove hands phase");
        m_phase = Phase::RemoveHands;

        m_StartTime = ctl.t();

        m_leftHandTargetRobot.translation() = m_safeLeftHandPositionRobot;
        m_leftHandTargetRobot.rotation()    = m_leftHandGraspOrientationRobot;
        m_leftGripperTask->set_ef_pose(m_leftHandTargetRobot);

        m_rightHandTargetRobot.translation() = m_safeRightHandPositionRobot;
        m_rightHandTargetRobot.rotation()    = m_rightHandGraspOrientationRobot;
        m_rightGripperTask->set_ef_pose(m_rightHandTargetRobot);

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

    ctl.gui()->removeElement({"GraspMoveBox"}, "Next Phase");
    ctl.gui()->removeElement({"GraspMoveBox"}, "Left gripper distance to box and speed");
    ctl.gui()->removeElement({"GraspMoveBox"}, "Right gripper distance to box and speed");


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

Eigen::Matrix3d GraspMoveBox::toXYPlane(Eigen::Matrix3d rotationMatrix)
{
    Eigen::Vector3d euler = mc_rbdyn::rpyFromMat(rotationMatrix);
    euler.x()             = 0.0;
    euler.y()             = 0.0;
    return mc_rbdyn::rpyToMat(euler);
}

sva::PTransformd GraspMoveBox::toHorizonAlignedPoseWorld(sva::PTransformd poseRobot, sva::PTransformd robotPoseWorld)
{
    double          angle(mc_rbdyn::rpyFromMat(robotPoseWorld.rotation()).z());
    Eigen::Matrix3d rotation   = Eigen::Matrix3d::Identity();
    rotation.block<2, 2>(0, 0) = Eigen::Rotation2Dd(-angle).toRotationMatrix();
    sva::PTransformd horizonAlignedRobotPoseWorld(rotation, robotPoseWorld.translation());

    sva::PTransformd horizonAlignedPoseWorld = poseRobot * horizonAlignedRobotPoseWorld;

    // mc_rtc::log::info(
    //                   "\npR: {}-{}\nrpW: {}-{}\npHAW: {}-{}",
    //                   poseRobot.translation().transpose(),
    //                   mc_rbdyn::rpyFromMat(poseRobot.rotation()).transpose(),
    //                   robotPoseWorld.translation().transpose(),
    //                   mc_rbdyn::rpyFromMat(robotPoseWorld.rotation()).transpose(),
    //                   horizonAlignedPoseWorld.translation().transpose(),
    //                   mc_rbdyn::rpyFromMat(horizonAlignedPoseWorld.rotation()).transpose()
    //                  );

    return horizonAlignedPoseWorld;
}

Eigen::Vector3d GraspMoveBox::toPose2DRobot(Eigen::Vector3d PoseWorld, sva::PTransformd robotPoseWorld)
{
    double          angle(mc_rbdyn::rpyFromMat(robotPoseWorld.rotation()).z());
    Eigen::Matrix2d rotation = Eigen::Rotation2Dd(angle).toRotationMatrix();

    Eigen::Vector2d relativePosition(
            PoseWorld.x() - robotPoseWorld.translation().x(), PoseWorld.y() - robotPoseWorld.translation().y());
    relativePosition = rotation * relativePosition;

    Eigen::Vector3d relativePose(relativePosition.x(), relativePosition.y(), PoseWorld.z() - angle);

    return relativePose;
}

EXPORT_SINGLE_STATE("GraspMoveBox", GraspMoveBox)
