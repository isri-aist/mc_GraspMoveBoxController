#include "ShoulderOrientation.hpp"
#include "../DemoController.h"
#include "./utils.h"

#include <BaselineWalkingController/CentroidalManager.h>
#include <SpaceVecAlg/PTransform.h>
#include <mc_tasks/CoMTask.h>
#include <mc_tasks/TransformTask.h>

void ShoulderOrientation::configure(const mc_rtc::Configuration & config)
{
    m_config.load(config);

    mc_rtc::log::info("ShoulderOrientation:\n{}", m_config.dump(true, true));

    m_leftReferenceFrame  = m_config("leftReferenceFrame", std::string("L_SHOULDER_P_LINK"));
    m_rightReferenceFrame = m_config("rightReferenceFrame", std::string("R_SHOULDER_P_LINK"));

    m_leftShoulderFrame   = m_config("leftShoulderFrame", std::string("L_SHOULDER_Y_LINK"));
    m_rightShoulderFrame  = m_config("rightShoulderFrame", std::string("R_SHOULDER_Y_LINK"));

    m_leftShoulderActiveJoints  = m_config("leftShoulderActiveJoints", std::vector<std::string>{"L_SHOULDER_Y"});
    m_rightShoulderActiveJoints = m_config("rightShoulderActiveJoints", std::vector<std::string>{"R_SHOULDER_Y"});

    m_stiffness          = m_config("stiffness", 2.0);
    m_weight             = m_config("weight", 2000.0);
    m_leftShoulderAngle  = m_config("leftShoulderAngle", 5.0 * M_PI / 180.0);
    m_rightShoulderAngle = m_config("rightShoulderAngle", -5.0 * M_PI / 180.0);
}

void ShoulderOrientation::start(mc_control::fsm::Controller & ctl_)
{
    auto & ctl = static_cast<DemoController&>(ctl_);

    // for (auto &j : ctl.robot().mb().joints()) mc_rtc::log::info("{}", j.name());

    m_leftElbowOrientationTask =
        std::make_shared<mc_tasks::OrientationTask>(ctl.robot().frame(m_leftShoulderFrame), m_stiffness, m_weight);
    ctl.solver().addTask(m_leftElbowOrientationTask);

    m_rightElbowOrientationTask =
        std::make_shared<mc_tasks::OrientationTask>(ctl.robot().frame(m_rightShoulderFrame), m_stiffness, m_weight);
    ctl.solver().addTask(m_rightElbowOrientationTask);

    updateOrientationTask(ctl_);
    addToGui(ctl_);
}

bool ShoulderOrientation::run(mc_control::fsm::Controller & ctl_)
{
    updateOrientationTask(ctl_);

    // is meant to run parallel
    return true;
}

void ShoulderOrientation::teardown(mc_control::fsm::Controller & ctl_)
{
    auto & ctl = static_cast<DemoController&>(ctl_);

    removeFromGui(ctl_);

    ctl.solver().removeTask(m_leftElbowOrientationTask);
    ctl.solver().removeTask(m_rightElbowOrientationTask);
}

void ShoulderOrientation::updateOrientationTask(mc_control::fsm::Controller & ctl_) const
{
    auto & ctl = static_cast<DemoController&>(ctl_);

    m_leftElbowOrientationTask->frame_ = ctl.robot().frame(m_leftShoulderFrame);
    m_leftElbowOrientationTask->stiffness(m_stiffness);
    m_leftElbowOrientationTask->weight(m_weight);

    m_rightElbowOrientationTask->frame_ = ctl.robot().frame(m_rightShoulderFrame);
    m_rightElbowOrientationTask->stiffness(m_stiffness);
    m_rightElbowOrientationTask->weight(m_weight);

    sva::PTransformd leftReferenceFrame  = ctl.robot().frame(m_leftReferenceFrame).position();
    sva::PTransformd rightReferenceFrame = ctl.robot().frame(m_rightReferenceFrame).position();

    const Eigen::Matrix3d leftRotation  = sva::RotX(m_leftShoulderAngle);
    const Eigen::Matrix3d rightRotation = sva::RotX(m_rightShoulderAngle);

    m_leftElbowOrientationTask->orientation(leftRotation * leftReferenceFrame.rotation());
    m_rightElbowOrientationTask->orientation(rightRotation * rightReferenceFrame.rotation());
}

void ShoulderOrientation::addToGui(mc_control::fsm::Controller & ctl_)
{
    auto & ctl = static_cast<DemoController&>(ctl_);

    ctl.gui()->addElement(
                          {"GMB", "Shoulder Orientation"},
                          mc_rtc::gui::NumberInput(
                                                   "Stiffness",
                                                   [this]
                                                   {
                                                       return m_stiffness;
                                                   },
                                                   [this](double value)
                                                   {
                                                       m_stiffness = value;
                                                   }
                                                  ),
                          mc_rtc::gui::NumberInput(
                                                   "Weight",
                                                   [this]
                                                   {
                                                       return m_weight;
                                                   },
                                                   [this](double value)
                                                   {
                                                       m_weight = value;
                                                   }
                                                  )
                         );

    ctl.gui()->addElement(
                          {"GMB", "Shoulder Orientation"},
                          mc_rtc::gui::NumberInput(
                                                   "Left Shoulder Angle",
                                                   [this]()
                                                   {
                                                       return m_leftShoulderAngle;
                                                   },
                                                   [this](double angle)
                                                   {
                                                       m_leftShoulderAngle = angle;
                                                   }
                                                  ),
                          mc_rtc::gui::NumberInput(
                                                   "Right Shoulder Angle",
                                                   [this]()
                                                   {
                                                       return m_rightShoulderAngle;
                                                   },
                                                   [this](double angle)
                                                   {
                                                       m_rightShoulderAngle = angle;
                                                   }
                                                  )
                         );

    ctl.gui()->addElement(
                          {"GMB", "Shoulder Orientation"},
                          mc_rtc::gui::ComboInput(
                                                  "Left Shoulder Controlled Frame",
                                                  ctl.robot().frames(),
                                                  [this]
                                                  {
                                                      return m_leftShoulderFrame;
                                                  },
                                                  [this](const std::string & frame)
                                                  {
                                                      m_leftShoulderFrame = frame;
                                                  }
                                                 ),
                          mc_rtc::gui::ComboInput(
                                                  "Left Shoulder Reference Frame",
                                                  ctl.robot().frames(),
                                                  [this]
                                                  {
                                                      return m_leftReferenceFrame;
                                                  },
                                                  [this](const std::string & frame)
                                                  {
                                                      m_leftReferenceFrame = frame;
                                                  }
                                                 ),
                          mc_rtc::gui::ComboInput(
                                                  "Right Shoulder Controlled Frame",
                                                  ctl.robot().frames(),
                                                  [this]
                                                  {
                                                      return m_rightShoulderFrame;
                                                  },
                                                  [this](const std::string & frame)
                                                  {
                                                      m_rightShoulderFrame = frame;
                                                  }
                                                 ),
                          mc_rtc::gui::ComboInput(
                                                  "Right Shoulder Reference Frame",
                                                  ctl.robot().frames(),
                                                  [this]
                                                  {
                                                      return m_rightReferenceFrame;
                                                  },
                                                  [this](const std::string & frame)
                                                  {
                                                      m_rightReferenceFrame = frame;
                                                  }
                                                 )
                         );

    std::vector<std::string> availableJointsNames;
    for (const auto & j : ctl.robot().mb().joints())
    {
        availableJointsNames.push_back(j.name());
    }

    ctl.gui()->addElement(
                          {"GMB", "Shoulder Orientation"},
                          mc_rtc::gui::ComboInput(
                                                  "Left Task Active Joint",
                                                  availableJointsNames,
                                                  [this]
                                                  {
                                                      return m_leftShoulderActiveJoints[0];
                                                  },
                                                  [this, &ctl](const std::string & joint)
                                                  {
                                                      m_leftShoulderActiveJoints = {joint};
                                                      ctl.solver().removeTask(m_leftElbowOrientationTask);
                                                      m_leftElbowOrientationTask->selectActiveJoints(
                                                           m_leftShoulderActiveJoints
                                                          );
                                                      ctl.solver().addTask(m_leftElbowOrientationTask);
                                                  }
                                                 ),
                          mc_rtc::gui::ComboInput(
                                                  "Right Task Active Joint",
                                                  availableJointsNames,
                                                  [this]
                                                  {
                                                      return m_rightShoulderActiveJoints[0];
                                                  },
                                                  [this, &ctl](const std::string & joint)
                                                  {
                                                      m_rightShoulderActiveJoints = {joint};
                                                      ctl.solver().removeTask(m_rightElbowOrientationTask);
                                                      m_rightElbowOrientationTask->selectActiveJoints(
                                                           m_rightShoulderActiveJoints
                                                          );
                                                      ctl.solver().addTask(m_rightElbowOrientationTask);
                                                  }
                                                 )
                         );

    ctl.gui()->addElement(
                          {"GMB", "Shoulder Orientation"},
                          mc_rtc::gui::Transform(
                                                 "Left Reference Frame Pose",
                                                 [this, &ctl]()
                                                 {
                                                     return ctl.robot().frame(m_leftReferenceFrame).position();
                                                 }
                                                ),
                          mc_rtc::gui::Transform(
                                                 "Right Reference Frame Pose",
                                                 [this, &ctl]()
                                                 {
                                                     return ctl.robot().frame(m_rightReferenceFrame).position();
                                                 }
                                                )
                         );
}

void ShoulderOrientation::removeFromGui(mc_control::fsm::Controller & ctl_)
{
    auto & ctl = static_cast<DemoController&>(ctl_);
    ctl.gui()->removeCategory({"GMB", "Shoulder Orientation"});
}

EXPORT_SINGLE_STATE("ShoulderOrientation", ShoulderOrientation)
