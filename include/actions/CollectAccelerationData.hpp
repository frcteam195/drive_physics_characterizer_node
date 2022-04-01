#pragma once

#include <vector>
#include "physics/DriveCharacterization.hpp"
#include "ck_utilities/CKTimer.hpp"

class CollectAccelerationData
{
public:
    CollectAccelerationData(std::vector<ck::physics::AccelerationDataPoint>& data, bool highGear, bool reverse, bool turn);
    void start();
    void update(double leftRPM, double rightRPM);
    bool isFinished();
    void done();

private:
    static constexpr double kPower = 0.45;
    static constexpr double kTotalTime = 2.25;

    std::vector<ck::physics::AccelerationDataPoint> *mAccelerationData;
    bool mTurn;
    bool mReverse;
    bool mHighGear;

    bool mFinished;
    double mPrevVelocity;
    double mPrevTime;

    ck::ElapsedTimer eTimer;
};