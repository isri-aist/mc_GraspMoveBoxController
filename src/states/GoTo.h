#pragma once

#include <Eigen/Geometry>
#include <mc_control/fsm/State.h>

struct GoTo : mc_control::fsm::State
{
    bool run(mc_control::fsm::Controller & ctl_) override;
    void start(mc_control::fsm::Controller & ctl_) override;
    void teardown(mc_control::fsm::Controller & ctl_) override;
    void configure(const mc_rtc::Configuration & config) override;

    protected:
        Eigen::Vector3d m_destinationPoseWorld;
        Eigen::Vector3d computeRelativePose(Eigen::Vector3d poseWorld, sva::PTransformd robotPoseWorld);
};
