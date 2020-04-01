//
// Created by zeid on 2/27/20.
//

#ifndef SRC_ROBOT_CONTROLLER_H
#define SRC_ROBOT_CONTROLLER_H


#include "sensor.h"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <ros/ros.h>
#include <stdarg.h>
#include <tf/transform_listener.h>
#include <iostream>
#include <string>
#include <initializer_list>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <osrf_gear/VacuumGripperControl.h>
#include <osrf_gear/VacuumGripperState.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
// #include <osrf_gear/Proximity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
class RobotController{
public:
    RobotController(std::string arm_id);
    ~RobotController();
    bool Planner();
    void Execute();
    void GoToTarget(std::initializer_list<geometry_msgs::Pose> list, int f);
    void GoToTarget(const geometry_msgs::Pose& pose,int f);
    void SendRobotHome();
    void SendRobotPosition(std::vector<double> pose);
    bool DropPart(geometry_msgs::Pose pose);
    void GripperToggle(const bool& state);
    void GripperCallback(const osrf_gear::VacuumGripperState::ConstPtr& grip);
    void GripperStateCheck(geometry_msgs::Pose pose);
    bool PickPart(geometry_msgs::Pose& part_pose);
    bool PickPartconveyor(std::string);
    //     void break_beam_callback_(const osrf_gear::Proximity::ConstPtr &);
    // int readBeam();
    // bool beam_;


private:
    ros::NodeHandle robot_controller_nh_;
    moveit::planning_interface::MoveGroupInterface::Options robot_controller_options;
    ros::ServiceClient gripper_client_;
    ros::NodeHandle gripper_nh_;
    ros::Subscriber gripper_subscriber_;
    // ros::Subscriber break_beam_subscriber_;
    AriacSensorManager beam;
    tf::TransformListener robot_tf_listener_;
    tf::StampedTransform robot_tf_transform_;
    tf::TransformListener agv_tf_listener_;
    tf::StampedTransform agv_tf_transform_;
    geometry_msgs::Pose target_pose_;
    geometry_msgs::Pose conveyor_part;

    moveit::planning_interface::MoveGroupInterface robot_move_group_;
    moveit::planning_interface::MoveGroupInterface::Plan robot_planner_;
    osrf_gear::VacuumGripperControl gripper_service_;
    osrf_gear::VacuumGripperState gripper_status_;

    std::string object;
    bool plan_success_;
    std::vector<double> home_joint_pose_;
    std::vector<double> bin_drop_pose_;
    std::vector<double> belt_drop_pose_;
    std::vector<double> conveyor;

    geometry_msgs::Pose home_cart_pose_;
    geometry_msgs::Quaternion fixed_orientation_;
    geometry_msgs::Quaternion fixed_end_orientation_;
    geometry_msgs::Quaternion conveyor_fixed_orientation_;
    geometry_msgs::Quaternion temp_orientation_;
    geometry_msgs::Pose agv_position_;
    geometry_msgs::Pose robot_pose_;
    std::vector<double> end_position_;
    double offset_;
    double roll_def_,pitch_def_,yaw_def_;
    tf::Quaternion q;
    int counter_;
    bool gripper_state_, drop_flag_,drop,pick;
};
#endif //SRC_ROBOT_CONTROLLER_H
