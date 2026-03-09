#include "GoTo.h"
#include "../DemoController.h"

#include <BaselineWalkingController/FootManager.h>

void GoTo::configure(const mc_rtc::Configuration &config)
{
    FootstepPlannerState::configure(config);

    mc_rtc::log::info("GoTo:\n{}", config.dump(true, true));
    config("autoStart", m_autoStart);
}

void GoTo::start(mc_control::fsm::Controller &ctl_)
{
    FootstepPlannerState::start(ctl_);

    auto &ctl = static_cast<DemoController &>(ctl_);

    Eigen::Vector3d relativePose = computeRelativePose(m_destinationPoseWorld, ctl.robot().posW());

    auto start = [&ctl, this, relativePose]
    {
        goalFootMidpose_ = {m_destinationPoseWorld.x(), m_destinationPoseWorld.y(), m_destinationPoseWorld.z()};
        triggered_       = true;
        m_planning       = true;
    };

    if (m_autoStart)
        start();

    else
        ctl.gui()->addElement({"GraspMoveBox"}, mc_rtc::gui::Button("Start", [start] { start(); }));
}

bool GoTo::run(mc_control::fsm::Controller &ctl_)
{
    auto &ctl = static_cast<DemoController &>(ctl_);

    if (m_planning) ctl.gui()->removeElement({"GraspMoveBox"}, "Start");

    if (m_planning && footstepPlanner_->solution_.is_solved) m_started = true;

    if (!m_started || !ctl.footManager_->footstepQueue().empty()) return false;

    output("OK");
    return true;
}

void GoTo::teardown(mc_control::fsm::Controller &ctl_)
{
    FootstepPlannerState::teardown(ctl_);
}

Eigen::Vector3d GoTo::computeRelativePose(Eigen::Vector3d PoseWorld, sva::PTransformd robotPoseWorld)
{
    double          angle(mc_rbdyn::rpyFromMat(robotPoseWorld.rotation()).z());
    Eigen::Matrix2d rotation = Eigen::Rotation2Dd(angle).toRotationMatrix();

    Eigen::Vector2d relativePosition(
            PoseWorld.x() - robotPoseWorld.translation().x(), PoseWorld.y() - robotPoseWorld.translation().y());
    relativePosition = rotation.transpose() * relativePosition;

    Eigen::Vector3d relativePose(relativePosition.x(), relativePosition.y(), PoseWorld.z() - angle);

    mc_rtc::log::info(
            "RobotW: {}-{}, DestinationW: {}, DestinationR: {}",
            robotPoseWorld.translation(),
            mc_rbdyn::rpyFromMat(robotPoseWorld.rotation()),
            PoseWorld,
            relativePose);

    return relativePose;
}
