#pragma once

#include <vector>
#include "physics/DriveCharacterization.hpp"
#include "ck_utilities/CKTimer.hpp"
#include "ros/ros.h"


class CollectVelocityData
{
public:
    CollectVelocityData(ros::NodeHandle* node, std::vector<ck::physics::VelocityDataPoint>& data, bool highGear, bool reverse, bool turn);
    void start();
    void update(double leftRPM, double rightRPM);
    bool isFinished();
    void done();

private:
    static constexpr double kMaxPower = 0.25;
    static constexpr double kRampRate = 0.02;

    std::vector<ck::physics::VelocityDataPoint> *mVelocityData;
    bool mTurn;
    bool mReverse;
    bool mHighGear;

    bool mFinished = false;
    double mStartTime = 0.0;

    ck::ElapsedTimer eTimer;
    ros::NodeHandle* mNode;
};