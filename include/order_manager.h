#pragma once

#include <list>
#include <map>
#include <string>
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_listener.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <osrf_gear/LogicalCameraImage.h>
#include <osrf_gear/Order.h>

#include "ariac_order_part.h"
#include "robot_controller.h"

//struct Order{
//	std::string shipment_type;
//	std::string agv_id;
//	std::vector< std::pair<std::string, geometry_msgs::Pose> > products;
//
//	friend std::ostream& operator << (std::ostream& os, const Order& order){
//		os << "shipment_type : " << order.shipment_type << "\n";
//		os << "agv_id : " << order.agv_id << "\n";
//		for(auto it = order.products.begin(); it!=order.products.end(); ++it){
//			os << "product type : " << it->first << "\n";
//			os << "product pose : " << it->second << "\n";
//		}
//		return os;
//	}
//};

class AriacOrderManager {

private:
	ros::NodeHandle* order_manager_nh_;
	ros::Subscriber order_subscriber_;
	std::vector<osrf_gear::Order> received_orders_;
//	std::map<std::string, Order> all_orders_; // create a copy of orders as you read so that it can be used in the future.
	std::map< std::string, std::map< std::string, std::vector<AriacPart> > > orders_;	// copy of order passed from competition
	std::map< std::string, std::vector<AriacPart> > all_order_parts_;
	std::map< std::string, std::vector<AriacPart> > belt_order_parts_;
	std::map< std::string, std::vector<AriacPart> > bin_order_parts_;
	std::map< std::string, std::map< std::string, std::vector<geometry_msgs::Pose> > >* bin_all_parts_sensors_;

	std::vector<std::string> product_type;
	ros::Subscriber wayPoint_subscriber;

	// AriacSensorManager camera_;
	RobotController arm1_;
	//    RobotController arm2_;
	tf::TransformListener part_tf_listener_;
	std::pair<std::string, geometry_msgs::Pose> product_type_pose_;
	std::string object;
	std::map<std::string, std::vector<std::string>> product_frame_list_;
	osrf_gear::Order order_;
	bool taskPending__;
	bool isBinCameraOn_;
	bool isOrderSegregated_;
public:
	AriacOrderManager(ros::NodeHandle *);
	~AriacOrderManager();
	void OrderCallback(const osrf_gear::Order::ConstPtr&);
	void ExecuteOrder();
	std::string GetProductFrame(std::string);
	std::map<std::string, std::list<std::pair<std::string,geometry_msgs::Pose>>> GetOrder();
	bool PickAndPlace(std::pair<std::string,geometry_msgs::Pose>,int );
	std::vector<std::string> getProductType();
	void readOrder();
	void segregateOrder();
	void SubmitAGV(int);
	ros::NodeHandle* getnode();
	void setBinCameraStatus(bool);
	bool isOrderSegregated() const;
	void setAllBinParts(std::map< std::string, std::map< std::string, std::vector< geometry_msgs::Pose > > >*);
	void assignCurrentPose(std::vector<geometry_msgs::Pose>, std::vector<AriacPart>&);
	void pathplanningCallback(const geometry_msgs::TransformStamped&);
};
