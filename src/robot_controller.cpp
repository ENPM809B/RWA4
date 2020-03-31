//
// Created by zeid on 2/27/20.
//
#include <tf/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include "../include/robot_controller.h"

/**
 * Constructor for the robot
 * Class attributes are initialized in the constructor init list
 * You can instantiate another robot by passing the correct parameter to the constructor
 */
RobotController::RobotController(std::string arm_id) : robot_controller_nh_("/ariac/"+arm_id), robot_controller_options("manipulator",
		"/ariac/"+arm_id+"/robot_description", robot_controller_nh_), robot_move_group_(robot_controller_options) {

	ROS_WARN(">>>>> RobotController");

	// setting parameters of planner
	robot_move_group_.setPlanningTime(20);
	robot_move_group_.setNumPlanningAttempts(10);
	robot_move_group_.setPlannerId("RRTConnectkConfigDefault");
	robot_move_group_.setMaxVelocityScalingFactor(0.9);
	robot_move_group_.setMaxAccelerationScalingFactor(0.9);
	// robot_move_group_.setEndEffector("moveit_ee");
	robot_move_group_.allowReplanning(true);


	//--These are joint positions used for the home position
	//	home_joint_pose_ = {0.0, 3.1, -1.1, 1.9, 3.9, 4.7, 0};
//	home_joint_pose_ = {-0.25, 0.0,  -0.7,  1.6, 3.9, -1.59, 0.126};
	home_joint_pose_ =  {0.1, 3.14,  -2.7,-1.0, 2.1, -1.59, 0.126};
	//home_joint_pose_ = {4.78, 0.09, -2.39, 3.14, -0.88, 1.51, 0};
	//-- offset used for picking up parts
	//-- For the pulley_part, the offset is different since the pulley is thicker
	offset_ = 0.025;

	//--topic used to get the status of the gripper
	gripper_subscriber_ = gripper_nh_.subscribe(
			"/ariac/arm1/gripper/state", 10, &RobotController::GripperCallback, this);


	SendRobotHome();

	robot_tf_listener_.waitForTransform("arm1_linear_arm_actuator", "arm1_ee_link",
			ros::Time(0), ros::Duration(10));
	robot_tf_listener_.lookupTransform("/arm1_linear_arm_actuator", "/arm1_ee_link",
			ros::Time(0), robot_tf_transform_);


	fixed_orientation_.x = robot_tf_transform_.getRotation().x();
	fixed_orientation_.y = robot_tf_transform_.getRotation().y();
	fixed_orientation_.z = robot_tf_transform_.getRotation().z();
	fixed_orientation_.w = robot_tf_transform_.getRotation().w();

	tf::quaternionMsgToTF(fixed_orientation_,q);
	tf::Matrix3x3(q).getRPY(roll_def_,pitch_def_,yaw_def_);


	end_position_ = {1.5, 1.5, -0.9, 1.9, 3.1, -1.59, 0.126};
	//	end_position_[0] = 2.2;
	//    end_position_[1] = 4.5;
	//    end_position_[2] = 1.2;
//    end_pose_.position.x = 0.0;
//    end_pose_.position.y = 0.0;
//    end_pose_.position.z = 0.0;
//    end_pose_.orientation = fixed_orientation_;



	robot_tf_listener_.waitForTransform("world", "arm1_ee_link", ros::Time(0),
			ros::Duration(10));
	robot_tf_listener_.lookupTransform("/world", "/arm1_ee_link", ros::Time(0),
			robot_tf_transform_);

	home_cart_pose_.position.x = robot_tf_transform_.getOrigin().x();
	home_cart_pose_.position.y = robot_tf_transform_.getOrigin().y();
	home_cart_pose_.position.z = robot_tf_transform_.getOrigin().z();
	home_cart_pose_.orientation.x = robot_tf_transform_.getRotation().x();
	home_cart_pose_.orientation.y = robot_tf_transform_.getRotation().y();
	home_cart_pose_.orientation.z = robot_tf_transform_.getRotation().z();
	home_cart_pose_.orientation.w = robot_tf_transform_.getRotation().w();

	agv_tf_listener_.waitForTransform("world", "kit_tray_1",
			ros::Time(0), ros::Duration(10));
	agv_tf_listener_.lookupTransform("/world", "/kit_tray_1",
			ros::Time(0), agv_tf_transform_);
	agv_position_.position.x = agv_tf_transform_.getOrigin().x();
	agv_position_.position.y = agv_tf_transform_.getOrigin().y();
	agv_position_.position.z = agv_tf_transform_.getOrigin().z() + 4 * offset_;

	gripper_client_ = robot_controller_nh_.serviceClient<osrf_gear::VacuumGripperControl>(
			"/ariac/arm1/gripper/control");
	counter_ = 0;
	drop_flag_ = false;
}

RobotController::~RobotController() {}

/**
 *
 * @return
 */
bool RobotController::Planner() {
	ROS_INFO_STREAM("Planning started...");
	if (robot_move_group_.plan(robot_planner_) ==
			moveit::planning_interface::MoveItErrorCode::SUCCESS) {
		plan_success_ = true;
		ROS_INFO_STREAM("Planner succeeded!");
	} else {
		plan_success_ = false;
		ROS_WARN_STREAM("Planner failed!");
	}

	return plan_success_;
}


void RobotController::Execute() {
	ros::AsyncSpinner spinner(4);
	spinner.start();
	if (this->Planner()) {
		robot_move_group_.move();
		ros::Duration(0.02).sleep();
	}
}

void RobotController::GoToTarget(const geometry_msgs::Pose& pose) {
	target_pose_.orientation = fixed_orientation_;
	target_pose_.position = pose.position;
	ros::AsyncSpinner spinner(4);
	robot_move_group_.setPoseTarget(target_pose_);
	spinner.start();
	if (this->Planner()) {
		ROS_INFO_STREAM("Point success");
		robot_move_group_.move();
		ros::Duration(0.02).sleep();
	}
	ROS_INFO_STREAM("Point reached...");
}

//void RobotController::GotoTarget(const geometry_msgs::Pose& pose) {
//	target_pose_.orientation = fixed_orientation_;
//	target_pose_.position = pose.position;
////	ros::AsyncSpinner spinner(4);
//	robot_move_group_.setPoseTarget(target_pose_);
////	spinner.start();
//		ROS_INFO_STREAM("Point success");
//		robot_move_group_.move();
//		ros::Duration(1.5).sleep();
//	ROS_INFO_STREAM("Point reached...");
//}

void RobotController::GoToTarget(
		std::initializer_list<geometry_msgs::Pose> list) {
	ros::AsyncSpinner spinner(4);
	spinner.start();

	std::vector<geometry_msgs::Pose> waypoints;
	for (auto i : list) {
		i.orientation.x = fixed_orientation_.x;
		i.orientation.y = fixed_orientation_.y;
		i.orientation.z = fixed_orientation_.z;
		i.orientation.w = fixed_orientation_.w;
		waypoints.emplace_back(i);
	}

	moveit_msgs::RobotTrajectory traj;
	auto fraction =
			robot_move_group_.computeCartesianPath(waypoints, 0.01, 0.0, traj, true);

	ROS_WARN_STREAM("Fraction: " << fraction * 100);
	ros::Duration(5.0).sleep();

	robot_planner_.trajectory_ = traj;

	//if (fraction >= 0.3) {
	robot_move_group_.execute(robot_planner_);
	ros::Duration(5.0).sleep();
	//    } else {
	//        ROS_ERROR_STREAM("Safe Trajectory not found!");
	//    }
}

void RobotController::SendRobotHome() {
	// ros::Duration(2.0).sleep();
	robot_move_group_.setJointValueTarget(home_joint_pose_);
	// this->execute();
	ros::AsyncSpinner spinner(4);
	spinner.start();
	if (this->Planner()) {
		robot_move_group_.move();
		ros::Duration(1.5).sleep();
	}

	ros::Duration(2.0).sleep();
}
void RobotController::GoToEnd(){
	std::vector<double> intermidiate_position = {0.5, 3.14,  -2.7,-1.0, 2.1, -1.59, 0.126};
	GoToPose(intermidiate_position);
	GoToPose(end_position_);
	GripperToggle(false);

}

void RobotController::GoToPose(const std::vector<double> & pose ) {
	robot_move_group_.setJointValueTarget(pose);
	// this->execute();
	ros::AsyncSpinner spinner(4);
	spinner.start();
	if (this->Planner()) {
		robot_move_group_.move();
		ros::Duration(0.02).sleep();
	}

	ros::Duration(0.05).sleep();

}

void RobotController::GripperToggle(const bool& state) {
	gripper_service_.request.enable = state;
	gripper_client_.call(gripper_service_);
	ros::Duration(0.01).sleep();
	// if (gripper_client_.call(gripper_service_)) {
	if (gripper_service_.response.success) {
		ROS_INFO_STREAM("Gripper activated!");
	} else {
		ROS_WARN_STREAM("Gripper activation failed!");
	}
}

// bool RobotController::dropPart(geometry_msgs::Pose part_pose) {
//   counter_++;
//
//   pick = false;
//   drop = true;
//
//   ROS_WARN_STREAM("Dropping the part number: " << counter_);
//
//   // ROS_INFO_STREAM("Moving to end of conveyor...");
//   // robot_move_group_.setJointValueTarget(part_pose);
//   // this->execute();
//   // ros::Duration(1.0).sleep();
//   // this->gripper_state_check(part_pose);
//
//   if (drop == false) {
//     // ROS_INFO_STREAM("I am stuck here..." << object);
//     ros::Duration(2.0).sleep();
//     return drop;
//   }
//   ROS_INFO_STREAM("Dropping on AGV...");
//
//   // agv_position_.position.x -= 0.1;
//   // if (counter_ == 1) {
//   //   agv_position_.position.y -= 0.1;
//   // }
//   // if (counter_ >= 2) {
//   //   agv_position_.position.y += 0.1;
//   //   // agv_position_.position.x +=0.1;
//   // }
//
//   auto temp_pose = part_pose;
//   // auto temp_pose = agv_position_;
//   temp_pose.position.z += 0.35;
//   // temp_pose.position.y += 0.5;
//
//   // this->setTarget(part_pose);
//   // this->execute();
//   // ros::Duration(1.0).sleep();
//   this->goToTarget({temp_pose, part_pose});
//   ros::Duration(1).sleep();
//   ROS_INFO_STREAM("Actuating the gripper...");
//   this->gripperToggle(false);
//
//   // ROS_INFO_STREAM("Moving to end of conveyor...");
//   // robot_move_group_.setJointValueTarget(end_position_);
//   // this->execute();
//   // ros::Duration(1.0).sleep();
//
//   ROS_INFO_STREAM("Going to home...");
//   // this->sendRobotHome();
//   // temp_pose = home_cart_pose_;
//   // temp_pose.position.z -= 0.05;
//   this->goToTarget({temp_pose, home_cart_pose_});
//   return drop;
// }

bool RobotController::DropPart(geometry_msgs::Pose part_pose) {
	// counter_++;

	drop_flag_ = true;

	ros::spinOnce();
	ROS_INFO_STREAM("Placing phase activated...");

	if (gripper_state_){//--while the part is still attached to the gripper
		//--move the robot to the end of the rail
		ROS_INFO_STREAM("Moving towards AGV1...");
		robot_move_group_.setJointValueTarget(end_position_);
		this->Execute();
		ros::Duration(1.0).sleep();
		ROS_INFO_STREAM("Actuating the gripper...");
		this->GripperToggle(false);

		//        auto temp_pose = part_pose;
		//        temp_pose.position.z += 0.5;
		//        this->GoToTarget({temp_pose, part_pose});
		//        ros::Duration(5).sleep();
		//        ros::spinOnce();
		//
		//
		//        ROS_INFO_STREAM("Actuating the gripper...");
		//        this->GripperToggle(false);
		//
		//        ros::spinOnce();
		//        if (!gripper_state_) {
		//            ROS_INFO_STREAM("Going to home position...");
		//            this->GoToTarget({temp_pose, home_cart_pose_});
		//            ros::Duration(3.0).sleep();
		//        }
	}

	drop_flag_ = false;
	return gripper_state_;
}

void RobotController::GripperCallback(
		const osrf_gear::VacuumGripperState::ConstPtr& grip) {
	gripper_state_ = grip->attached;
}


bool RobotController::PickPart(geometry_msgs::Pose& part_pose) {
	// gripper_state = false;
	// pick = true;
	//ROS_INFO_STREAM("fixed_orientation_" << part_pose.orientation = fixed_orientation_);
	//ROS_WARN_STREAM("Picking the part...");

	ROS_INFO_STREAM("Moving to part...");
	part_pose.position.z = part_pose.position.z + offset_;
	auto temp_pose_1 = part_pose;
	temp_pose_1.position.z += 0.1;

	this->GoToTarget({temp_pose_1, part_pose});

	ROS_INFO_STREAM("Actuating the gripper..." << part_pose.position.z);
	this->GripperToggle(true);
	ros::spinOnce();
	while (!gripper_state_) {
		part_pose.position.z -= 0.01;
		this->GoToTarget({temp_pose_1, part_pose});
                temp_pose_1 = part_pose;
		ROS_INFO_STREAM("Actuating the gripper...");
		this->GripperToggle(true);
		ros::spinOnce();
	}

	ROS_INFO_STREAM("Going to waypoint...");
	this->GoToTarget(temp_pose_1);
	return gripper_state_;
}

bool RobotController::isPartAttached(){
	return gripper_state_;
}



geometry_msgs::Pose RobotController::getHomeCartPose(){
	return home_cart_pose_;
}

//geometry_msgs::Pose RobotController::convertToArmBaseFrame( const geometry_msgs::PoseStamped& pose_msg) {
//	geometry_msgs::PoseStamped arm_base_part_pose_stamped;
////	geometry_msgs::Pose arm_base_part_pose;
//	 try {
//	robot_tf_listener_.transformPose ("/arm1_linear_arm_actuator", pose_msg, arm_base_part_pose_stamped);
//	 } catch(tf::TransformException& ex) {
//         ROS_ERROR_STREAM("Unable to transform object from frame "  << ex.what());
//     }
//
//	arm_base_part_pose_stamped.pose.orientation.x = fixed_orientation_.x;
//	arm_base_part_pose_stamped.pose.orientation.y = fixed_orientation_.y;
//	arm_base_part_pose_stamped.pose.orientation.z = fixed_orientation_.z;
//	arm_base_part_pose_stamped.pose.orientation.w = fixed_orientation_.w;
//	return arm_base_part_pose_stamped.pose;
//
//}

//
//geometry_msgs::Pose RobotController::convertToArmBaseFrame( const geometry_msgs::TransformStamped& t_stammed) {
//	//tf2_ros::TransformBroadcaster br_w_arm;
//	geometry_msgs::Pose arm_base_part_pose;
//	tf2_ros::TransformBroadcaster br_arm_part;
//	tf2_ros::Buffer tfBuffer;
//
//	tf2_ros::TransformListener tfListener(tfBuffer);
//	geometry_msgs::TransformStamped ts_w_part;
//	geometry_msgs::TransformStamped ts_arm_part;
//	ts_w_part.header.stamp = pose_msg.header.stamp;
//	ts_w_part.header.frame_id = "world";
//	ts_w_part.child_frame_id = "tracking_part";
//	ts_w_part.transform.translation.x = pose_msg.pose.position.x;
//	ts_w_part.transform.translation.y = pose_msg.pose.position.y;
//	ts_w_part.transform.translation.z = pose_msg.pose.position.z;
//	ts_w_part.transform.rotation.x = pose_msg.pose.orientation.x;
//	ts_w_part.transform.rotation.y = pose_msg.pose.orientation.y;
//	ts_w_part.transform.rotation.z = pose_msg.pose.orientation.z;
//	ts_w_part.transform.rotation.w = pose_msg.pose.orientation.w;
//	br_arm_part.sendTransform(ts_w_part);
//	ros::Duration(0.01).sleep();
//
////	tfBuffer.waitForTransform("arm1_linear_arm_actuator", "tracking_part",
////				ros::Time(0), ros::Duration(10));
//	try{
//		ts_arm_part = tfBuffer.lookupTransform("arm1_linear_arm_actuator", "tracking_part",
//				ros::Time(0));
//	}
//	catch (tf2::TransformException &ex) {
//		ROS_WARN("exception");
//		ROS_WARN("%s",ex.what());
//		ros::Duration(0.01).sleep();
//	}
//	arm_base_part_pose.position.x = ts_arm_part.transform.translation.x;
//	arm_base_part_pose.position.y = ts_arm_part.transform.translation.y;
//	arm_base_part_pose.position.z = ts_arm_part.transform.translation.z;
//	arm_base_part_pose.orientation.x = fixed_orientation_.x;
//	arm_base_part_pose.orientation.y = fixed_orientation_.y;
//	arm_base_part_pose.orientation.z = fixed_orientation_.z;
//	arm_base_part_pose.orientation.w = fixed_orientation_.w;
// return arm_base_part_pose;
//}
//
//geometry_msgs::Pose RobotController::convertToArmBaseFrame( ) {
//	//tf2_ros::TransformBroadcaster br_w_arm;
//	geometry_msgs::Pose arm_base_part_pose;
//	tf2_ros::Buffer tfBuffer;
//
//
//	tf2_ros::TransformListener tfListener(tfBuffer);
//	geometry_msgs::TransformStamped ts_arm_part;
//
//	try{
//		ts_arm_part = tfBuffer.lookupTransform("arm1_linear_arm_actuator", "logical_sensor_child",
//				ros::Time(0));
//	}
//	catch (tf2::TransformException &ex) {
//		ROS_WARN("exception");
//		ROS_WARN("%s",ex.what());
//		ros::Duration(0.01).sleep();
//	}
//	arm_base_part_pose.position.x = ts_arm_part.transform.translation.x;
//	arm_base_part_pose.position.y = ts_arm_part.transform.translation.y;
//	arm_base_part_pose.position.z = ts_arm_part.transform.translation.z;
//	arm_base_part_pose.orientation.x = fixed_orientation_.x;
//	arm_base_part_pose.orientation.y = fixed_orientation_.y;
//	arm_base_part_pose.orientation.z = fixed_orientation_.z;
//	arm_base_part_pose.orientation.w = fixed_orientation_.w;
// return arm_base_part_pose;
//}

