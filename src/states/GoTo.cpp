#include "GoTo.h"
#include "../DemoController.h"

#include <BaselineWalkingController/FootManager.h>

void GoTo::configure(const mc_rtc::Configuration & config) {}

void GoTo::start(mc_control::fsm::Controller & ctl_)
{
    auto & ctl = static_cast<DemoController&>(ctl_);

    Eigen::Vector3d relativePose = computeRelativePose(m_destinationPoseWorld, ctl.robot().posW());

    ctl.footManager_->reset();
    ctl.footManager_->walkToRelativePose(relativePose);
}

bool GoTo::run(mc_control::fsm::Controller & ctl_)
{
    auto & ctl = static_cast<DemoController&>(ctl_);

    if (!ctl.footManager_->footstepQueue().empty()) return false;

    output("OK");
    return true;
}

void GoTo::teardown(mc_control::fsm::Controller & ctl_) {}

Eigen::Vector3d GoTo::computeRelativePose(Eigen::Vector3d PoseWorld, sva::PTransformd robotPoseWorld)
{
    double          angle(mc_rbdyn::rpyFromMat(robotPoseWorld.rotation()).z());
    Eigen::Matrix2d rotation = Eigen::Rotation2Dd(angle).toRotationMatrix();

    Eigen::Vector2d relativePosition(
                                     PoseWorld.x() - robotPoseWorld.translation().x(),
                                     PoseWorld.y() - robotPoseWorld.translation().y()
                                    );
    relativePosition = rotation * relativePosition;

    Eigen::Vector3d relativePose(relativePosition.x(), relativePosition.y(), PoseWorld.z() - angle);

    return relativePose;
}
