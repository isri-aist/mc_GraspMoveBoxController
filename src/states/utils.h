#pragma once

#include <string>
#include <vector>

const std::vector<std::string> LeftArmJoints = {
        "L_SHOULDER_P",
        "L_SHOULDER_R",
        "L_SHOULDER_Y",
        "L_ELBOW_P",
        "L_ELBOW_Y",
        "L_WRIST_R",
        "L_WRIST_Y",
};

const std::vector<std::string> RightArmJoints = {
        "R_SHOULDER_P",
        "R_SHOULDER_R",
        "R_SHOULDER_Y",
        "R_ELBOW_P",
        "R_ELBOW_Y",
        "R_WRIST_R",
        "R_WRIST_Y",
};

enum BoxType
{
    BoxBase,
    BoxNoLid,
    BoxLid
};

enum BoxSide
{
    Left,
    Right
};

inline Eigen::Vector3d BoxOffsetFromRobotOffset(Eigen::Vector3d &robotOffset, BoxType boxType, BoxSide boxSide)
{
    switch (boxType)
    {
        case BoxBase:
        {
            mc_rtc::log::error_and_throw("box type not implemented yet");
        }
        case BoxLid:
        {
            mc_rtc::log::error_and_throw("box type not implemented yet");
        }
        case BoxNoLid:
        {
            if (boxSide == Left) return {robotOffset.z(), -robotOffset.x(), -robotOffset.y()};
            if (boxSide == Right) return {robotOffset.z(), robotOffset.x(), -robotOffset.y()};
        }
    }

    mc_rtc::log::error_and_throw("box type invalid");
}
