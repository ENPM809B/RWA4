#include <RobotController.h>

 /*
  * @brief
  *
  * @param
  *
  * @return
  */
RobotControl::RobotControl() :
  robot_controller_nh_("/ariac/arm1"),
  robot_controller_options("manipulator",
        "/ariac/arm1/robot_description",
        robot_controller_nh_),
  robot_move_group_(robot_controller_options) {
    ROS_WARN("<<<<< RobotController >>>>>");

    robot_move_group_.setPlanningTime(10);
    robot_move_group_.setNumPlanningAttempts(30);
    robot_move_group_.setPlannerId("RRTConnectkConfigDefault");
    robot_move_group_.setMaxVelocityScalingFactor(1.0);
    robot_move_group_.setMaxAccelerationScalingFactor(1.0);
    // robot_move_group_.setEndEffector("moveit_ee");
    robot_move_group_.allowReplanning(true);

    /* These are joint positions used for the home position
     * [0] = linear_arm_actuator
     * [1] = shoulder_pan_joint
     * [2] = shoulder_lift_joint
     * [3] = elbow_joint
     * [4] = wrist_1_joint
     * [5] = wrist_2_joint
     * [6] = wrist_3_joint
     */
    home_joint_pose_ = {0.47, 3.24, -2.41, -1.78, 5.8, -4.71, -1.13};
    // home_joint_pose_ = {0.00, 3.14, -2.00, 2.14, 4.54, -1.63, -1.13};


    //-- offset used for picking up parts
    //-- For the pulley_part, the offset is different since the pulley is thicker
    offset_ = 0.027;

    gripper_subscriber_ = gripper_nh_.subscribe(
            "/ariac/arm1/gripper/state", 10, &RobotControl::GripperCb, this);


    gripper_client_ = robot_controller_nh_.serviceClient<osrf_gear::VacuumGripperControl>(
            "/ariac/arm1/gripper/control");

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

  }

 /*
  * @brief
  *
  * @param
  *
  * @return
  */
void RobotControl::SendRobotHome() {
  robot_move_group_.setJointValueTarget(home_joint_pose_);
  this->GripperToggle(true);
  // this->execute();
  ros::AsyncSpinner spinner(4);
  spinner.start();
  if (this->Planner()) {
    robot_move_group_.move();
    ros::Duration(1.5).sleep();
  }

  ros::Duration(2.0).sleep();
}

bool RobotControl::Planner() {
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

void RobotControl::GripperCb(
        const osrf_gear::VacuumGripperState::ConstPtr& grip) {
    gripper_state_ = grip->attached;
}

void RobotControl::GripperToggle(const bool& state) {
    gripper_service_.request.enable = state;
    gripper_client_.call(gripper_service_);
    ros::Duration(1.0).sleep();
    // if (gripper_client_.call(gripper_service_)) {
    if (gripper_service_.response.success) {
        ROS_INFO_STREAM("Gripper activated!");
    } else {
        ROS_WARN_STREAM("Gripper activation failed!");
    }
}

void RobotControl::GoToTarget(geometry_msgs::Pose& pose) {
  pose.position.z = pose.position.z + 0.025;
  auto temp_pose_1 = pose;
  temp_pose_1.position.z += 0.3;
  std::initializer_list<geometry_msgs::Pose> list = {temp_pose_1, pose};
  // target_pose_.orientation = fixed_orientation_;
  // target_pose_.position = pose.position;
  // if (flag) {
  //   target_pose_.position.z += 0.2;
  // }
  ros::AsyncSpinner spinner(4);
  // robot_move_group_.setPoseTarget(target_pose_);
  spinner.start();
  // if (this->Planner()) {
  //   robot_move_group_.move();
  //   ros::Duration(1.5).sleep();
  // }
  // while (!gripper_state_) {
  //   flag = false;
  //   target_pose_.position.z -= 0.1;
  //   this->GoToTarget(target_pose_);
  //   ROS_INFO_STREAM("Actuating the gripper...");
  //   // this->GripperToggle(true);
  //   ros::spinOnce();
  // }
  std::vector<geometry_msgs::Pose> waypoints;
  for (auto i : list) {
    i.orientation.x = fixed_orientation_.x;
    i.orientation.y = fixed_orientation_.y;
    i.orientation.z = fixed_orientation_.z;
    i.orientation.w = fixed_orientation_.w;
    waypoints.emplace_back(i);
  }
  if (this->Planner()) {
    robot_move_group_.move();
    ros::Duration(1.5).sleep();
  }
  moveit_msgs::RobotTrajectory traj;
  auto fraction =
          robot_move_group_.computeCartesianPath(waypoints, 0.01, 0.0, traj, true);

  ROS_WARN_STREAM("Fraction: " << fraction);
  // ros::Duration(5.0).sleep();

  robot_planner_.trajectory_ = traj;

    //if (fraction >= 0.3) {
  robot_move_group_.execute(robot_planner_);
  ros::Duration(5.0).sleep();
  ROS_INFO_STREAM("Point reached...");
}

RobotControl::~RobotControl() {}
