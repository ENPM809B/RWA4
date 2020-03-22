#ifndef INCLUDE_ROBOTCONTROLLER_H
#define INCLUDE_ROBOTCONTROLLER_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <stdarg.h>
#include <osrf_gear/VacuumGripperControl.h>
#include <osrf_gear/VacuumGripperState.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <osrf_gear/AGVControl.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf/transform_listener.h>
#include <iostream>
#include <string>
#include <initializer_list>

class RobotControl {
private:
  ros::NodeHandle robot_controller_nh_;
  moveit::planning_interface::MoveGroupInterface::Options robot_controller_options;
  ros::ServiceClient gripper_client_;
  ros::NodeHandle gripper_nh_;
  ros::Subscriber gripper_subscriber_;

  tf::TransformListener robot_tf_listener_;
  tf::StampedTransform robot_tf_transform_;
  tf::TransformListener agv_tf_listener_;
  tf::StampedTransform agv_tf_transform_;

  geometry_msgs::Pose target_pose_;

  moveit::planning_interface::MoveGroupInterface robot_move_group_;
  moveit::planning_interface::MoveGroupInterface::Plan robot_planner_;

  osrf_gear::VacuumGripperControl gripper_service_;
  osrf_gear::VacuumGripperState gripper_status_;

  std::string object;
  bool plan_success_;
  std::vector<double> home_joint_pose_;
  geometry_msgs::Pose home_cart_pose_;
  geometry_msgs::Quaternion fixed_orientation_;
  geometry_msgs::Pose agv_position_;
  std::vector<double> end_position_;
  double offset_;
  double roll_def_,pitch_def_,yaw_def_;
  tf::Quaternion q;
  int counter_;
  bool gripper_state_, drop_flag_;
  bool flag = true;

public:
  RobotControl();
 /*
  * @brief
  *
  * @param
  *
  * @return
  */
  void SendRobotHome();

  bool Planner();

  void GripperCb(const osrf_gear::VacuumGripperState::ConstPtr& grip);

  void GripperToggle(const bool& state);

  void GoToTarget(geometry_msgs::Pose& pose);

  ~RobotControl();

};

#endif // INCLUDE_ROBOTCONTROLLER_H
