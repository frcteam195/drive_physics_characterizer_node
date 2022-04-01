#include "actions/CollectVelocityData.hpp"
#include <rio_control_node/Motor_Control.h>
#include <rio_control_node/Cal_Override_Mode.h>
#include <cmath>
#include "actions/DriveSetHelper.hpp"

CollectVelocityData::CollectVelocityData(std::vector<ck::physics::VelocityDataPoint>& data, bool highGear, bool reverse, bool turn)
{
    mVelocityData = &data;
    mHighGear = highGear;
    mReverse = reverse;
    mTurn = turn;
}

void CollectVelocityData::start()
{
    eTimer.start();
}

void CollectVelocityData::update(double leftRPM, double rightRPM)
{
    double percentPower = kRampRate * (eTimer.hasElapsed());
    if (percentPower > kMaxPower) {
        mFinished = true;
        return;
    }

    DriveSetHelper::getInstance().setDrivePercentOut((mReverse ? -1.0 : 1.0) * percentPower, (mReverse ? -1.0 : 1.0) * (mTurn ? -1.0 : 1.0) * percentPower);
    mVelocityData->push_back(ck::physics::VelocityDataPoint{
        (std::abs(leftRPM) + std::abs(rightRPM)) * ck::math::PI / 60.0, //velocity in rad/s
        percentPower * 12.0 //convert to volts
    });
}

bool CollectVelocityData::isFinished()
{
    return mFinished;
}

void CollectVelocityData::done()
{
    DriveSetHelper::getInstance().setDrivePercentOut(0,0);
}