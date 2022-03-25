#include "ros/ros.h"
#include "std_msgs/String.h"

#include <thread>
#include <string>
#include <mutex>
#include <vector>
#include <rio_control_node/Motor_Status.h>
#include <rio_control_node/Robot_Status.h>
#include "actions/DriveSetHelper.hpp"
#include "physics/DriveCharacterization.hpp"
#include "DriveCharacterizationParameters.hpp"
#include "actions/CollectVelocityData.hpp"
#include "actions/CollectAccelerationData.hpp"
#include "ck_utilities/ParameterHelper.hpp"
#include <atomic>
#include <drive_physics_characterizer_node/Drive_Characterization_Output.h>

ros::NodeHandle* node;
std::atomic<double> leftMotorRpm;
std::atomic<double> rightMotorRpm;

static bool begin_test = false;

void robotStatusCallback(const rio_control_node::Robot_Status &msg)
{
	if (msg.robot_state > 0)
	{
		begin_test = true;
	}
}

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

	ROS_INFO("Drive Characterization | Waiting for enable...");

	while (!begin_test)
	{
		rate.sleep();
	}

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
	static ros::Publisher drive_char_pub = node->advertise<drive_physics_characterizer_node::Drive_Characterization_Output>("/DriveCharacterizationOutput", 1);
	ros::Rate rate(50);
	while (ros::ok())
	{
		DriveSetHelper::getInstance().publishMessage(drive_char_pub);
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

	ros::Subscriber motorStatusSub = node->subscribe("MotorStatus", 10, motorStatusCallback);
	ros::Subscriber robotStatusSub = node->subscribe("RobotStatus", 1, robotStatusCallback);

	std::thread mDrivePublishThread(publishDrive);
	std::thread mDriveCharacterize(characterizeDrive);

	ros::spin();

	mDrivePublishThread.join();
	mDriveCharacterize.join();

	return 0;
}