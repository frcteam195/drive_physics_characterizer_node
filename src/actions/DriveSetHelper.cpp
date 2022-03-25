#include "actions/DriveSetHelper.hpp"
#include "DriveCharacterizationParameters.hpp"

DriveSetHelper::DriveSetHelper()
{
    mDriveCharMsg.characterizing_drive = true;
}

void DriveSetHelper::setDrivePercentOut(double leftMotorVal, double rightMotorVal)
{
    std::lock_guard<std::mutex> lock(mDriveHelperLock);
    mDriveCharMsg.left_drive_output = leftMotorVal;
    mDriveCharMsg.right_drive_output = rightMotorVal;
}

void DriveSetHelper::publishMessage(ros::Publisher& drive_char_pub)
{
    std::lock_guard<std::mutex> lock(mDriveHelperLock);
    drive_char_pub.publish(mDriveCharMsg);
}