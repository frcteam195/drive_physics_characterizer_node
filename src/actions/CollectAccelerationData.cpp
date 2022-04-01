#include "actions/CollectAccelerationData.hpp"
#include <rio_control_node/Motor_Control.h>
#include <rio_control_node/Cal_Override_Mode.h>
#include <cmath>
#include "actions/DriveSetHelper.hpp"
#include "ck_utilities/CKMath.hpp"

CollectAccelerationData::CollectAccelerationData(std::vector<ck::physics::AccelerationDataPoint>& data, bool highGear, bool reverse, bool turn)
{
    mAccelerationData = &data;
    mHighGear = highGear;
    mReverse = reverse;
    mTurn = turn;
}

void CollectAccelerationData::start()
{
    DriveSetHelper::getInstance().setDrivePercentOut((mReverse ? -1.0 : 1.0) * kPower, (mReverse ? -1.0 : 1.0) * (mTurn ? -1.0 : 1.0) * kPower);
    eTimer.start();
}

void CollectAccelerationData::update(double leftRPM, double rightRPM)
{
    double currentVelocity = (std::fabs(leftRPM) + std::fabs(rightRPM)) * ck::math::PI / 60.0;
    double currentTime = eTimer.hasElapsed();
    if (mPrevTime == 0)
    {
        mPrevTime = currentTime;
        mPrevVelocity = currentVelocity;
        return;
    }
    
    double acceleration = (currentVelocity - mPrevVelocity) / (currentTime - mPrevTime);
    //ignore accelerations that are too small
    if (acceleration < ck::math::kEpsilon) {
        mPrevTime = currentTime;
        mPrevVelocity = currentVelocity;
        return;
    }

    mAccelerationData->push_back(ck::physics::AccelerationDataPoint{
            currentVelocity, //convert velocity in rpms
            kPower * 12.0, //convert to volts
            acceleration
    });

    mPrevTime = currentTime;
    mPrevVelocity = currentVelocity;
}

bool CollectAccelerationData::isFinished()
{
    return eTimer.hasElapsed() > kTotalTime;
}

void CollectAccelerationData::done()
{
    DriveSetHelper::getInstance().setDrivePercentOut(0,0);
}