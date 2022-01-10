#pragma once

#include <vector>
#include "physics/DriveCharacterization.hpp"
#include "ck_utilities/CKTimer.hpp"

class CollectVelocityData
{
public:
    CollectVelocityData(std::vector<ck::physics::VelocityDataPoint>& data, bool highGear, bool reverse, bool turn);
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

    bool mFinished;

    ck::ElapsedTimer eTimer;
};