#include <osrf_gear/AGVControl.h>
#include <string>
#include <algorithm>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include "../include/order_manager.h"


//AriacOrderManager::AriacOrderManager(): arm1_{"arm1"}, arm2_{"arm2"}
AriacOrderManager::AriacOrderManager(ros::NodeHandle* nh): arm1_{"arm1"}
{
	order_manager_nh_ = nh;
	wayPoint_subscriber = order_manager_nh_->subscribe("/ariac/logical_sensor_4/tracking_object", 10, &AriacOrderManager::pathplanningCallback, this);
	order_subscriber_ = order_manager_nh_->subscribe("/ariac/orders", 10, &AriacOrderManager::OrderCallback, this);
	taskPending_ = true;
	isBinCameraOn_ = false;
	isOrderSegregated_ = false;
}

AriacOrderManager::~AriacOrderManager(){}


void AriacOrderManager::OrderCallback(const osrf_gear::Order::ConstPtr& order_msg) {
	ROS_WARN(">>>>> OrderCallback");

	// When ever you receive a new order add it.
	received_orders_.push_back(*order_msg);

	ROS_INFO_STREAM("No. of orders received " << received_orders_.size() << std::endl);

	readOrder();

	while ( not isBinCameraOn_ )
	{
		ROS_INFO_STREAM("Waiting for bin camera to start..." << std::endl);
		ros::Duration(1).sleep();
	}

	ROS_INFO_STREAM("Bin camera started..." << std::endl);

	segregateOrder();

}

void AriacOrderManager::readOrder(){

	ROS_INFO_STREAM("Processing order..." << std::endl);

//	for(const auto &order:received_orders_) {			// Later on make sure that we don't read from the beginning again. duplicate copies
		auto& order = received_orders_.back();			// read only the latest order
		auto order_id = order.order_id;						// Get order id
		auto shipments = order.shipments;					// get shipments

		for (const auto &shipment: shipments) {

			auto shipment_type = shipment.shipment_type;
			auto products = shipment.products;

			for (const auto &product: products) {

				AriacPart part(product.type, shipment.agv_id, product.pose); // create an AriacPart object

				orders_[order_id][shipment_type].push_back(part); // This is just to create a copy of the order for future use, if need be
				all_order_parts_[product.type].push_back(part);	  // This is adding all parts by part type
				product_type_pose_.first = product.type;
				product_type.push_back(product.type);
			}
		}
//	}

	// Print order parts processed
//	for(auto it=all_order_parts_.begin(); it!=all_order_parts_.end(); ++it)
//	{
//		ROS_INFO_STREAM("Parts processed are " << std::endl);
//		ROS_INFO_STREAM("There are " << it->second.size() << "parts of type " << it->first << std::endl);
//		for(auto jt=it->second.begin(); jt!=it->second.end(); ++jt){
//			ROS_INFO_STREAM("They are " << jt->getPartType() << " on agv " << jt->getPartAgvID()<< " with pose " << jt->getPartEndPose()<<"\n");
//		}
//	}
}

void AriacOrderManager::segregateOrder() {

	ROS_INFO_STREAM("Segregating order parts into bin parts and belt parts..." << std::endl);

	for(auto it=all_order_parts_.begin(); it!=all_order_parts_.end(); ++it ) { // for each part type in order list

		bool flag = false;
		auto& order_part_type = it->first;

		for(const auto& element : *bin_all_parts_sensors_ ) { // for each entry from sensor map<camera_id, map<part_type, vector<Pose>>>
			auto& cam_id  = element.first;
			auto& map_partType_poses = element.second;

			if( map_partType_poses.count(order_part_type) == 1 ){ // if part_type exists in cam_id's map ie in any of the bins
				if(it->second.size() <= map_partType_poses.at(order_part_type).size()) { // if part_type parts in order <= cam_id's bin parts
					assignCurrentPose(map_partType_poses.at(order_part_type), it->second);
					bin_order_parts_[order_part_type] = it->second;
					flag = true;
				}
				else {
					ROS_WARN_STREAM("HMMMM...not enough parts on the bin, so need to use conveyor belt.." << std::endl);
				}
			}
		}
		if(not flag) {// none of bin cameras read this part_type, so add these part_types to belt collection
			belt_order_parts_[order_part_type] = it->second;
		}
	}

	// set true as it needs to be monitored in sensor manager
	isOrderSegregated_ = true;

	ROS_INFO_STREAM("Products needed to be processed from belt are \n");
	for( auto& ele : belt_order_parts_ ) {
		ROS_INFO_STREAM("Part type " << ele.first << " , and number = " << ele.second.size() << std::endl);
	}

	ROS_INFO_STREAM("Products needed to be processed from bins are\ n");
	for( auto& ele : bin_order_parts_ ) {
		ROS_INFO_STREAM("Part type " << ele.first << " , and number = " << ele.second.size() << std::endl);
	}
}

void AriacOrderManager::assignCurrentPose(std::vector<geometry_msgs::Pose> binPoses, std::vector<AriacPart>& orderParts){

	auto min_length = std::min(binPoses.size(), orderParts.size()); // minimum can only be of length of orderparts
	for(int i=0; i<=min_length; ++i){
		orderParts[i].setPartCurrentPose(binPoses[i]);
	}
}

std::map< std::string, std::vector<AriacPart> >* AriacOrderManager::getConveyorBeltOrderParts() {
	return &belt_order_parts_;
}

void AriacOrderManager::setBinCameraStatus(bool status){
	isBinCameraOn_ = status;
}

void AriacOrderManager::setAllBinParts(std::map< std::string, std::map< std::string, std::vector< geometry_msgs::Pose > > >* from_sensor_manager){
	bin_all_parts_sensors_ = from_sensor_manager;
}

bool AriacOrderManager::isOrderSegregated() const {
	return isOrderSegregated_;
}

std::vector<std::string> AriacOrderManager::getProductType(){
	return product_type;
}


/**
 * @brief Get the product frame for a product type
 * @param product_type
 * @return
 */
std::string AriacOrderManager::GetProductFrame(std::string product_type) {
	//--Grab the last one from the list then remove it
	if (!product_frame_list_.empty()) {
		std::string frame = product_frame_list_[product_type].back();
		ROS_INFO_STREAM("Frame >>>> " << frame);
		product_frame_list_[product_type].pop_back();
		return frame;
	} else {
		ROS_ERROR_STREAM("No product frame found for " << product_type);
		ros::shutdown();
	}
}

//bool AriacOrderManager::PickAndPlace(const std::pair<std::string,geometry_msgs::Pose> product_type_pose, int agv_id) {
//    std::string product_type = product_type_pose.first;
//    ROS_WARN_STREAM("Product type >>>> " << product_type);
//    std::string product_frame = this->GetProductFrame(product_type);
//    ROS_WARN_STREAM("Product frame >>>> " << product_frame);
//    auto part_pose = camera_.GetPartPose("/world",product_frame);
//
//
//    if(product_type == "pulley_part")
//        part_pose.position.z += 0.08;
//    //--task the robot to pick up this part
//    bool failed_pick = arm1_.PickPart(part_pose);
//    ROS_WARN_STREAM("Picking up state " << failed_pick);
//    ros::Duration(0.5).sleep();
//
//    while(!failed_pick){
//        auto part_pose = camera_.GetPartPose("/world",product_frame);
//        failed_pick = arm1_.PickPart(part_pose);
//    }
//
//    //--get the pose of the object in the tray from the order
//    geometry_msgs::Pose drop_pose = product_type_pose.second;
//
//    geometry_msgs::PoseStamped StampedPose_in,StampedPose_out;
//
//    if(agv_id==1){
//        StampedPose_in.header.frame_id = "/kit_tray_1";
//        StampedPose_in.pose = drop_pose;
//        ROS_INFO_STREAM("StampedPose_int (" << StampedPose_in.pose.position.x <<","<< StampedPose_in.pose.position.y << "," << StampedPose_in.pose.position.z<<")");
//        part_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);
//        StampedPose_out.pose.position.z += 0.1;
//        StampedPose_out.pose.position.y -= 0.2;
//        ROS_INFO_STREAM("StampedPose_out (" << StampedPose_out.pose.position.x <<","<< StampedPose_out.pose.position.y << "," << StampedPose_out.pose.position.z<<")");
//
//    }
//    else{
//        StampedPose_in.header.frame_id = "/kit_tray_2";
//        StampedPose_in.pose = drop_pose;
//        //ROS_INFO_STREAM("StampedPose_in " << StampedPose_in.pose.position.x);
//        part_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);
//        StampedPose_out.pose.position.z += 0.1;
//        StampedPose_out.pose.position.y += 0.2;
//        //ROS_INFO_STREAM("StampedPose_out " << StampedPose_out.pose.position.x);
//    }
//    auto result = arm1_.DropPart(StampedPose_out.pose);
//
//    return result;
//}


//void AriacOrderManager::ExecuteOrder() {
//    ROS_WARN(">>>>>> Executing order...");
//    //scanned_objects_ = camera_.GetParts();
//
//    //-- used to check if pick and place was successful
//    bool pick_n_place_success{false};
//
//    std::list<std::pair<std::string,geometry_msgs::Pose>> failed_parts;
//
//    ros::spinOnce();
//    ros::Duration(1.0).sleep();
//    product_frame_list_ = camera_.get_product_frame_list();
//    for (const auto &order:received_orders_){
//        auto order_id = order.order_id;
//        auto shipments = order.shipments;
//        for (const auto &shipment: shipments){
//            auto shipment_type = shipment.shipment_type;
//            auto agv = shipment.agv_id.back();//--this returns a char
//            //-- if agv is any then we use AGV1, else we convert agv id to int
//            //--agv-'0' converts '1' to 1 and '2' to 2
//            int agv_id = (shipment.agv_id == "any") ? 1 : agv - '0';
//
//            auto products = shipment.products;
//            ROS_INFO_STREAM("Order ID: " << order_id);
//            ROS_INFO_STREAM("Shipment Type: " << shipment_type);
//            ROS_INFO_STREAM("AGV ID: " << agv_id);
//            for (const auto &product: products){
//                ros::spinOnce();
//                product_type_pose_.first = product.type;
//                product_type.push_back(product.type);
//                //ROS_INFO_STREAM("Product type: " << product_type_pose_.first);
//                product_type_pose_.second = product.pose;
//                ROS_INFO_STREAM("Product pose: " << product_type_pose_.second.position.x);
//                pick_n_place_success =  PickAndPlace(product_type_pose_, agv_id);
//                //--todo: What do we do if pick and place fails?
//            }
//            SubmitAGV(1);
//            ROS_INFO_STREAM("Submitting AGV 1");
//            int finish=1;
//        }
//
//
//    }
//}


void AriacOrderManager::SubmitAGV(int num) {
	std::string s = std::to_string(num);
	ros::ServiceClient start_client =
			order_manager_nh_->serviceClient<osrf_gear::AGVControl>("/ariac/agv"+s);
	if (!start_client.exists()) {
		ROS_INFO("Waiting for the client to be ready...");
		start_client.waitForExistence();
		ROS_INFO("Service started.");
	}

	osrf_gear::AGVControl srv;
	// srv.request.kit_type = "order_0_kit_0";
	start_client.call(srv);

	if (!srv.response.success) {
		ROS_ERROR_STREAM("Service failed!");
	} else
		ROS_INFO("Service succeeded.");
}


ros::NodeHandle* AriacOrderManager::getnode() {
	return order_manager_nh_;
}

void AriacOrderManager::pathplanningCallback(const geometry_msgs::TransformStamped& msg) {

	if(taskPending_) {
		ROS_INFO("Picking up part from the conveyor belt");
		double threshold_z = 0.1;
		double threshold_y = 0.35;
		geometry_msgs::Pose arm_base_part_pose;
		arm_base_part_pose.position.x= msg.transform.translation.x;
		arm_base_part_pose.position.y= msg.transform.translation.y-0.2;
		arm_base_part_pose.position.z= msg.transform.translation.z;
		arm_base_part_pose.orientation.x= msg.transform.rotation.x;
		arm_base_part_pose.orientation.y= msg.transform.rotation.y;
		arm_base_part_pose.orientation.z= msg.transform.rotation.z;
		arm_base_part_pose.orientation.w = msg.transform.rotation.w;
		//	if(count ==0) {
		//	ROS_INFO_STREAM("isPartAttached status" << arm1_.isPartAttached());
		if(!arm1_.isPartAttached()) {
			//		ROS_INFO("part not attached");
			//		ROS_INFO_STREAM(msg.transform.translation.x<<","<< msg.transform.translation.y<<","<< msg.transform.translation.z);
			arm1_.GoToTarget(arm_base_part_pose);
			//		ROS_INFO("going toward part");
			ROS_INFO_STREAM("gap: "<<arm1_.getHomeCartPose().position.z- msg.transform.translation.z << ","<< arm1_.getHomeCartPose().position.y- msg.transform.translation.y);
			if(arm1_.getHomeCartPose().position.z- msg.transform.translation.z < threshold_z &&
					arm1_.getHomeCartPose().position.y- msg.transform.translation.y < threshold_y) {
				//			arm1_.GoToTarget(arm_base_part_pose);
				// arm1_.PickPart(arm_base_part_pose);
				arm1_.GripperToggle(true);
				arm_base_part_pose.position.z += 0.2;
				arm_base_part_pose.position.y += 0.5;
				arm1_.GoToTarget(arm_base_part_pose);
			}
		} else {
			arm1_.GoToEnd();
			taskPending_ = false;
		}
	}

}
