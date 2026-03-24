#include "LookAtBox.hpp"

#include <SpaceVecAlg/SpaceVecAlg>
#include "../DemoController.h"

sva::PTransformd computeBoxInCam(const sva::PTransformd &CamInWorld, const sva::PTransformd &BoxInWorld)
{
    sva::PTransformd pose = BoxInWorld * CamInWorld.inv();
    return pose;
}

void LookAtBox::configure(const mc_rtc::Configuration &config)
{
    config("stiffness", m_stiffness);
    config("weight", m_weight);
    config("objectName", m_objectName);
    config("cameraControlFrame", m_cameraControlFrame);
}

void LookAtBox::start(mc_control::fsm::Controller &ctl_)
{
    auto &ctl = static_cast<DemoController &>(ctl_);

    m_gazeTask = std::make_shared<mc_tasks::GazeTask>(ctl.robot().frame(m_cameraControlFrame), m_stiffness, m_weight);
    m_gazeTask->selectActiveJoints({
            "HEAD_Y",
            "HEAD_P",
    });
    ctl.solver().addTask(m_gazeTask);

    ctl.gui()->addElement(
            {"GMB", "LookAtBox"},
            mc_rtc::gui::Transform(
                    "camera", [this, &ctl] { return ctl.robot().frame(m_cameraControlFrame).position(); }),
            mc_rtc::gui::Transform("box", [this, &ctl] { return ctl.robot(m_objectName).posW(); }),
            mc_rtc::gui::Transform(
                    "box in cam",
                    [this, &ctl]
                    {
                        return computeBoxInCam(
                                ctl.robot().frame(m_cameraControlFrame).position(), ctl.robot(m_objectName).posW());
                    }));

    ctl.logger().addLogEntry("gaze_error", [this] { return m_error; });
}

bool LookAtBox::run(mc_control::fsm::Controller &ctl_)
{
    auto &ctl = static_cast<DemoController &>(ctl_);

    // Project object pose in the configured camera frame (x forward, z up)
    const auto objectPosW = ctl.robot(m_objectName).posW();
    const auto cameraPosW = ctl.robot().frame(m_cameraControlFrame).position();
    const auto objectPosC = computeBoxInCam(cameraPosW, objectPosW);

    m_error = {-objectPosC.translation().z(), objectPosC.translation().y()};
    m_gazeTask->error(m_error);

    return true;
}

void LookAtBox::teardown(mc_control::fsm::Controller &ctl_)
{
    auto &ctl = static_cast<DemoController &>(ctl_);
    ctl.solver().removeTask(m_gazeTask);

    ctl.gui()->removeCategory({"GMB", "LookAtBox"});
    ctl.logger().removeLogEntry("gaze_error");
}

EXPORT_SINGLE_STATE("LookAtBox", LookAtBox)
