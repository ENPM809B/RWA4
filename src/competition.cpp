#include "competition.h"

#include <osrf_gear/LogicalCameraImage.h>
#include <osrf_gear/Order.h>
#include <osrf_gear/Proximity.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <trajectory_msgs/JointTrajectory.h>

Competition::Competition() : current_score_(0), arm_1_has_been_zeroed_(false), arm_2_has_been_zeroed_(false){

//	manager_(&comp_nh_);

	// Subscribe to the '/ariac/current_score' topic.
	current_score_subscriber = comp_nh_.subscribe("/ariac/current_score", 10, &Competition::current_score_callback, this);

	// Subscribe to the '/ariac/competition_state' topic.
	competition_state_subscriber = comp_nh_.subscribe("/ariac/competition_state", 10, &Competition::competition_state_callback, this);

	// publish joint values/trajectory to each arm
	arm_1_joint_trajectory_publisher_ = comp_nh_.advertise<trajectory_msgs::JointTrajectory>("/ariac/arm1/arm/command", 10);

	arm_2_joint_trajectory_publisher_ = comp_nh_.advertise<trajectory_msgs::JointTrajectory>("/ariac/arm2/arm/command", 10);

	StartCompetition();
}

Competition::~Competition(){

}

/// Called when a new message is received.
void Competition::current_score_callback(const std_msgs::Float32::ConstPtr & msg) {
	if (msg->data != current_score_)
	{
		ROS_INFO_STREAM("Score: " << msg->data);
	}
	current_score_ = msg->data;
}

/// Called when a new message is received.
void Competition::competition_state_callback(const std_msgs::String::ConstPtr & msg) {
	if (msg->data == "done" && competition_state_ != "done")
	{
		ROS_INFO("Competition ended.");
	}
	competition_state_ = msg->data;
}

/// Called when a new JointState message is received.

void Competition::arm_1_joint_state_callback(
		const sensor_msgs::JointState::ConstPtr & joint_state_msg)
{
	ROS_INFO_STREAM_THROTTLE(10,
			"Joint States arm 1 (throttled to 0.1 Hz):\n" << *joint_state_msg);
	// ROS_INFO_STREAM("Joint States:\n" << *joint_state_msg);
	arm_1_current_joint_states_ = *joint_state_msg;
	if (!arm_1_has_been_zeroed_) {
		arm_1_has_been_zeroed_ = true;
		ROS_INFO("Sending arm to zero joint positions...");
		send_arm_to_zero_state(arm_1_joint_trajectory_publisher_);
	}
}

void Competition::arm_2_joint_state_callback(
		const sensor_msgs::JointState::ConstPtr & joint_state_msg)
{
	ROS_INFO_STREAM_THROTTLE(10,
			"Joint States arm 2 (throttled to 0.1 Hz):\n" << *joint_state_msg);
	// ROS_INFO_STREAM("Joint States:\n" << *joint_state_msg);
	arm_2_current_joint_states_ = *joint_state_msg;
	if (!arm_2_has_been_zeroed_) {
		arm_2_has_been_zeroed_ = true;
		ROS_INFO("Sending arm 2 to zero joint positions...");
		send_arm_to_zero_state(arm_2_joint_trajectory_publisher_);
	}
}

/// Create a JointTrajectory with all positions set to zero, and command the arm.
void Competition::send_arm_to_zero_state(ros::Publisher & joint_trajectory_publisher) {
	// Create a message to send.
	trajectory_msgs::JointTrajectory msg;

	// Fill the names of the joints to be controlled.
	// Note that the vacuum_gripper_joint is not controllable.
	msg.joint_names.clear();
	msg.joint_names.push_back("shoulder_pan_joint");
	msg.joint_names.push_back("shoulder_lift_joint");
	msg.joint_names.push_back("elbow_joint");
	msg.joint_names.push_back("wrist_1_joint");
	msg.joint_names.push_back("wrist_2_joint");
	msg.joint_names.push_back("wrist_3_joint");
	msg.joint_names.push_back("linear_arm_actuator_joint");
	// Create one point in the trajectory.
	msg.points.resize(1);
	// Resize the vector to the same length as the joint names.
	// Values are initialized to 0.
	msg.points[0].positions.resize(msg.joint_names.size(), 0.0);
	// How long to take getting to the point (floating point seconds).
	msg.points[0].time_from_start = ros::Duration(0.001);
	ROS_INFO_STREAM("Sending command:\n" << msg);
	joint_trajectory_publisher.publish(msg);
}

void Competition::StartCompetition() {
	ROS_INFO("Competition function.");
	// Create a Service client for the correct service, i.e. '/ariac/start_competition'.
	ros::ServiceClient start_client =
			comp_nh_.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
	// If it's not already ready, wait for it to be ready.
	// Calling the Service using the client before the server is ready would fail.
	if (!start_client.exists()) {
		ROS_INFO("Waiting for the competition to be ready...");
		start_client.waitForExistence();
		ROS_INFO("Competition is now ready.");
	}
	ROS_INFO("Requesting competition start...");
	std_srvs::Trigger srv;  // Combination of the "request" and the "response".
	start_client.call(srv);  // Call the start Service.
	if (!srv.response.success) {  // If not successful, print out why.
		ROS_ERROR_STREAM("Failed to start the competition: " << srv.response.message);
	} else {
		ROS_INFO("Competition started!");
	}
}

void Competition::EndCompetition() {
	ros::ServiceClient start_client =
			comp_nh_.serviceClient<std_srvs::Trigger>("/ariac/end_competition");

	if (!start_client.exists()) {
		ROS_INFO("Waiting for the competition to be End...");
		start_client.waitForExistence();
		ROS_INFO("Competition  now Ended.");
	}
	ROS_INFO("Requesting competition End...");
	std_srvs::Trigger srv;
	start_client.call(srv);
	if (!srv.response.success) {
		ROS_ERROR_STREAM(
				"Failed to End the competition: " << srv.response.message);
	} else {
		ROS_INFO("Competition Ended!");
	}
}
