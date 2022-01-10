#include "ros/ros.h"
#include "std_msgs/String.h"

#include <thread>
#include <string>
#include <mutex>
#include <rio_control_node/Motor_Control.h>
#include "actions/DriveSetHelper.hpp"
#include "DriveCharacterizationParameters.hpp"

#define STR_PARAM(s) #s
#define CKSP(s) ckgp( STR_PARAM(s) )
static const std::string nodeParamPrefix = "/drive_physics_characterizer_node/";
std::string ckgp(std::string instr)
{
	std::string retVal = nodeParamPrefix;
	retVal += instr;
	return retVal;
}

ros::NodeHandle* node;

void publishDrive()
{
	static ros::Publisher robot_drive_pub = node->advertise<rio_control_node::Motor_Control>("MotorTuningControl", 1);
	ros::Rate rate(50);
	while (ros::ok())
	{
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

	std::thread mDrivePublishThread();

	ros::spin();
	return 0;
}