#pragma once

#include <list>
#include <map>
#include <string>

#include <ros/ros.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <osrf_gear/LogicalCameraImage.h>
#include <osrf_gear/Proximity.h>
#include <order_manager.h>
#include <ariac_part.h>

//class AriacOrderManager;

class AriacSensorManager {

private:
	ros::NodeHandle sensor_nh_;
	AriacOrderManager* orderManager;

	ros::Subscriber camera_1_subscriber_;
	ros::Subscriber camera_4_subscriber_;
	ros::Subscriber camera_5_subscriber_;
	ros::Subscriber breakbeam_subscriber;
	ros::Subscriber quality_control_camera_subscriber_;

	bool cam_1_processed_;
	bool cam_2_processed_;
	std::map< std::string, std::map<std::string, std::vector<geometry_msgs::Pose> > > binParts_;
//	std::map< std::string, std::vector<geometry_msgs::Pose> > > accumulated_binParts_;

	osrf_gear::Model*  tracking_part;

	tf::TransformListener camera_tf_listener_;
	tf::StampedTransform camera_tf_transform_;
	osrf_gear::LogicalCameraImage current_parts_4_;
	bool object_detected = false;
	std::map<std::string, std::vector<geometry_msgs::Pose>> part_list_;
	std::map<std::string, std::vector<std::string>> product_frame_list_;
	ros::Publisher transform_publisher;

public:
	AriacSensorManager(AriacOrderManager *);
	~AriacSensorManager();
	void convertPose(const geometry_msgs::Pose, geometry_msgs::TransformStamped &);
	void convertPose(const geometry_msgs::Pose, geometry_msgs::Pose &);
	void binlogicalCameraCallback(const osrf_gear::LogicalCameraImage::ConstPtr &, std::string); // common callback service for both cameras
	void binlogicalCameraCallback1(const osrf_gear::LogicalCameraImage::ConstPtr &);
	void binlogicalCameraCallback2(const osrf_gear::LogicalCameraImage::ConstPtr &);
	void beltlogicalCameraCallback(const osrf_gear::LogicalCameraImage::ConstPtr &);
	void getBinParts(const osrf_gear::LogicalCameraImage::ConstPtr&, std::string );
	void breakBeamCallback(const osrf_gear::Proximity::ConstPtr &);
	bool isObjectDetectedBreakBeam();
};
