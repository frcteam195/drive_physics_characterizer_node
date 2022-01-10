#include "actions/CollectVelocityData.hpp"
#include <rio_control_node/Motor_Control.h>
#include <rio_control_node/Cal_Override_Mode.h>
#include <cmath>
#include "actions/DriveSetHelper.hpp"

CollectVelocityData::CollectVelocityData(ros::NodeHandle* node, std::vector<ck::physics::VelocityDataPoint>& data, bool highGear, bool reverse, bool turn)
{
    mNode = node;
    mVelocityData = &data;
    mHighGear = highGear;
    mReverse = reverse;
    mTurn = turn;
}

void CollectVelocityData::start()
{
    //TODO: Set High Gear
    eTimer.start();
}

void CollectVelocityData::update(double leftRPM, double rightRPM)
{
    double percentPower = kRampRate * (eTimer.hasElapsed());
    if (percentPower > kMaxPower) {
        mFinished = true;
        return;
    }

    //Set Drive power
    //mDrive.setOpenLoop(new DriveSignal((mReverse ? -1.0 : 1.0) * percentPower, (mReverse ? -1.0 : 1.0) * (mTurn ? -1.0 : 1.0) * percentPower));

    mVelocityData->push_back(ck::physics::VelocityDataPoint{
        (std::abs(leftRPM) + std::abs(rightRPM)) * M_PI / 60.0, //convert velocity to radians per second 
        percentPower * 12.0 //convert to volts
    });
}

bool CollectVelocityData::isFinished()
{
    return mFinished;
}

void CollectVelocityData::done()
{
    //Turn off Drive
    //Flush data
}