#include "actions/DriveSetHelper.hpp"
#include "DriveCharacterizationParameters.hpp"

DriveSetHelper::DriveSetHelper()
{
    motorCtrl.motors.push_back(leftMotor);
    motorCtrl.motors.push_back(rightMotor);
}

void DriveSetHelper::setDrivePercentOut(double leftMotorVal, double rightMotorVal)
{
    std::lock_guard<std::mutex> lock(mDriveHelperLock);
    leftMotor.output_value = leftMotorVal;
    rightMotor.output_value = rightMotorVal;
}

void DriveSetHelper::publishMessage(ros::Publisher &robot_drive_pub)
{
    std::lock_guard<std::mutex> lock(mDriveHelperLock);
    robot_drive_pub.publish(motorCtrl);
}

void DriveSetHelper::setParameters(DriveCharacterizationParameters &params)
{
    leftMotor.id = params.left_master_id;
    leftMotor.controller_type = params.motor_type;
    leftMotor.control_mode = rio_control_node::Motor::PERCENT_OUTPUT;
    leftMotor.output_value = 0;
    leftMotor.arbitrary_feedforward = 0;

    rightMotor.id = params.right_master_id;
    rightMotor.controller_type = params.motor_type;
    rightMotor.control_mode = rio_control_node::Motor::PERCENT_OUTPUT;
    rightMotor.output_value = 0;
    rightMotor.arbitrary_feedforward = 0;
}