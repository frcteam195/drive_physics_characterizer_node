#include "ros/ros.h"
#include "std_msgs/String.h"

#include <thread>
#include <string>
#include <mutex>
#include <vector>
#include <rio_control_node/Motor_Control.h>
#include <rio_control_node/Motor_Status.h>
#include <rio_control_node/Cal_Override_Mode.h>
#include "actions/DriveSetHelper.hpp"
#include "physics/DriveCharacterization.hpp"
#include "DriveCharacterizationParameters.hpp"
#include "actions/CollectVelocityData.hpp"
#include "actions/CollectAccelerationData.hpp"
#include <atomic>

#define STR_PARAM(s) #s
#define CKSP(s) ckgp( STR_PARAM(s) )
std::string ckgp(std::string instr)
{
	std::string retVal = ros::this_node::getName();
	retVal += "/" + instr;
	return retVal;
}

ros::NodeHandle* node;
std::atomic<double> leftMotorRpm;
std::atomic<double> rightMotorRpm;

void motorStatusCallback(const rio_control_node::Motor_Status &msg)
{
	for (auto it = msg.motors.begin(); it != msg.motors.end(); it++ )
	{
		if (it->id == 1)
		{
			leftMotorRpm = it->sensor_velocity;
		}
		if (it->id == 4)
		{
			rightMotorRpm = it->sensor_velocity;
		}
	}
}

void characterizeDrive()
{
	bool angularCharacterization = false;

	std::vector<ck::physics::VelocityDataPoint> velocityData;
	std::vector<ck::physics::AccelerationDataPoint> accelerationData;
	ros::Rate rate(50);

	ROS_INFO("Beginning Characterization of Velocity...");

	CollectVelocityData cvd(velocityData, false, false, angularCharacterization);
	cvd.start();
	while (!cvd.isFinished())
	{
		cvd.update(leftMotorRpm, rightMotorRpm);
		rate.sleep();
	}
	cvd.done();

	ROS_INFO("Characterization of Velocity Completed!");

	ROS_INFO("Sleeping for 20 seconds so the robot can be reset.");
	std::this_thread::sleep_for(std::chrono::seconds(20));

	ROS_INFO("Beginning Characterization of Acceleration...");

	CollectAccelerationData cad(accelerationData, false, false, angularCharacterization);
	cad.start();
	while (!cad.isFinished())
	{
		cad.update(leftMotorRpm, rightMotorRpm);
		rate.sleep();
	}
	cad.done();

	ROS_INFO("Characterization of Acceleration Completed!");

	ck::physics::CharacterizationConstants constants = ck::physics::DriveCharacterization::characterizeDrive(velocityData, accelerationData);
	ROS_INFO("Characterization Constants | Kv: %lf, Ka: %lf, Ks: %lf", constants.kv, constants.ka, constants.ks);
	ros::shutdown();
}

void publishDrive()
{
	static ros::Publisher robot_drive_pub = node->advertise<rio_control_node::Motor_Control>("MotorTuningControl", 1);
	static ros::Publisher tuning_override_pub = node->advertise<rio_control_node::Cal_Override_Mode>("OverrideMode", 1);
	static rio_control_node::Cal_Override_Mode overrideModeMsg;
	overrideModeMsg.operation_mode = rio_control_node::Cal_Override_Mode::TUNING_PIDS;
	ros::Rate rate(50);
	while (ros::ok())
	{
		tuning_override_pub.publish(overrideModeMsg);
		DriveSetHelper::getInstance().publishMessage(robot_drive_pub);
		rate.sleep();
	}
}

int main(int argc, char **argv)
{
	/**
	 * The ros::init() function needs to see argc and argv so that it can perform
	 * any ROS arguments and name remapping that were provided at the command line.
	 * For programmatic remappings you can use a different version of init() which takes
	 * remappings directly, but for most command-line programs, passing argc and argv is
	 * the easiest way to do it.  The third argument to init() is the name of the node.
	 *
	 * You must call one of the versions of ros::init() before using any other
	 * part of the ROS system.
	 */
	ros::init(argc, argv, "drive_physics_characterizer_node");

	ros::NodeHandle n;

	node = &n;

	DriveCharacterizationParameters params;
	bool required_params_found = true;
	required_params_found &= n.getParam(CKSP(left_master_id), params.left_master_id);
	required_params_found &= n.getParam(CKSP(right_master_id), params.right_master_id);
	required_params_found &= n.getParam(CKSP(motor_type), params.motor_type);
	if (!required_params_found)
	{
		ROS_ERROR("Missing required parameters. Please check the list and make sure all required parameters are included");
		return 1;
	}

	DriveSetHelper::getInstance().setParameters(params);

	ros::Subscriber motorStatusSub = node->subscribe("MotorStatus", 10, motorStatusCallback);

	std::thread mDrivePublishThread(publishDrive);
	std::thread mDriveCharacterize(characterizeDrive);

	ros::spin();

	mDrivePublishThread.join();
	mDriveCharacterize.join();

	return 0;
}