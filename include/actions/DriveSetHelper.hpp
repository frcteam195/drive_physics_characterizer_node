#pragma once
#include "ros/ros.h"
#include "ck_utilities/Singleton.hpp"
#include <mutex>
#include "DriveCharacterizationParameters.hpp"
#include <drive_physics_characterizer_node/Drive_Characterization_Output.h>

class DriveSetHelper : public Singleton<DriveSetHelper>
{
    friend Singleton;
public:
    void setDrivePercentOut(double leftMotorVal, double rightMotorVal);
    void publishMessage(ros::Publisher &drive_char_pub);

private:
    DriveSetHelper();

    drive_physics_characterizer_node::Drive_Characterization_Output mDriveCharMsg;

    std::mutex mDriveHelperLock;
};