#pragma once
#include "ros/ros.h"
#include <rio_control_node/Motor_Control.h>
#include <rio_control_node/Motor.h>
#include "ck_utilities/Singleton.hpp"
#include <mutex>
#include "DriveCharacterizationParameters.hpp"

class DriveSetHelper : public Singleton<DriveSetHelper>
{
    friend Singleton;
public:
    void setDrivePercentOut(double leftMotorVal, double rightMotorVal);
    void publishMessage(ros::Publisher &robot_drive_pub);
    void setParameters(DriveCharacterizationParameters &params);

private:
    DriveSetHelper();

    rio_control_node::Motor_Control motorCtrl;
    rio_control_node::Motor leftMotor;
    rio_control_node::Motor rightMotor;

    std::mutex mDriveHelperLock;
};