//
// Created by zeid on 2/27/20.
//
#include "../include/sensor.h"

AriacSensorManager::AriacSensorManager(AriacOrderManager* obj): orderManager(obj)
{
	//	sensor_nh_ = orderManager->getnode();
	ROS_INFO_STREAM(">>>>> Subscribing to logical cameras and control sensors");

	camera_1_subscriber_ = sensor_nh_.subscribe("/ariac/logical_camera_1", 10, &AriacSensorManager::binlogicalCameraCallback1, this);

	camera_4_subscriber_ = sensor_nh_.subscribe("/ariac/logical_camera_4", 10, &AriacSensorManager::beltlogicalCameraCallback, this);

	camera_5_subscriber_ = sensor_nh_.subscribe("/ariac/logical_camera_5", 10, &AriacSensorManager::binlogicalCameraCallback2, this);

//	quality_control_camera_subscriber_ = sensor_nh_.subscribe("/ariac/quality_control_sensor_1", 10 , &AriacSensorManager::qualityControlSensor1Callback, this);

	//	breakbeam_subscriber = sensor_nh_->subscribe("/ariac/break_beam_1", 10,	&AriacSensorManager::breakBeamCallback,this);
	tracking_part= nullptr;

	// To publish transformed pose of each part w.r.t world
	transform_publisher = sensor_nh_.advertise<geometry_msgs::TransformStamped>("/ariac/logical_sensor_4/tracking_object", 10);

	cam_1_processed_ = false;
	cam_2_processed_ = false;
}


AriacSensorManager::~AriacSensorManager() {}


void AriacSensorManager::convertPose(const geometry_msgs::Pose pose, geometry_msgs::TransformStamped& transformStamped ){
	transformStamped.transform.translation.x = pose.position.x;
	transformStamped.transform.translation.y = pose.position.y;
	transformStamped.transform.translation.z = pose.position.z;
	transformStamped.transform.rotation.x = pose.orientation.x;
	transformStamped.transform.rotation.y = pose.orientation.y;
	transformStamped.transform.rotation.z = pose.orientation.z;
	transformStamped.transform.rotation.w = pose.orientation.w;
}

void AriacSensorManager::convertPose( const geometry_msgs::Pose src, geometry_msgs::Pose & dstn){
	dstn.position.x = src.position.x;
	dstn.position.y = src.position.y;
	dstn.position.z = src.position.z;
	dstn.orientation.x = src.orientation.x;
	dstn.orientation.y = src.orientation.y;
	dstn.orientation.z = src.orientation.z;
	dstn.orientation.w = src.orientation.w;
}

void AriacSensorManager::binlogicalCameraCallback1(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg ){

	binlogicalCameraCallback(image_msg, "cam1");
}


void AriacSensorManager::binlogicalCameraCallback2(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg){

	binlogicalCameraCallback(image_msg, "cam2");
}

void AriacSensorManager::binlogicalCameraCallback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg, std::string cam_name){
// all bin part is empty fill the all bin part for segregate purpose
    getBinParts(image_msg, cam_name);

    if( cam_1_processed_ && cam_2_processed_ ) {
//    	ROS_INFO_STREAM("Finished Reading both bin camera parts..." << std::endl);
    	orderManager->setAllBinParts(&binParts_);   // get binparts from one camera for now?
    	orderManager->setBinCameraStatus(true);		// set bin camera processed so that order manager can start segregating parts.

    	//print what each camera sees - debugging ---> working as expected
//    	for(const auto& cam_id : binParts_ ) {
//    		ROS_INFO_STREAM("Camera " << cam_id.first << " sees \n");
//    		for(const auto& part_type : cam_id.second ) {
//    			ROS_INFO_STREAM("part_type " << part_type.first << " in number = " << part_type.second.size());
//    		}
//    	}
//    	cam_1_processed_ = false;
//    	cam_2_processed_ = false;
    }




}

void AriacSensorManager::getBinParts( const osrf_gear::LogicalCameraImage::ConstPtr& image_msg, std::string cam_name ){
	// get bin parts as seen from both cameras
	// Here, you need to get the Pose of the part w.r.t camera and also make a transform for camera w.r.t world

	auto sensor_pose = image_msg->pose;					// get sensor frame wrt world
	geometry_msgs::TransformStamped tf_sensor_wrt_world;	// T matrix for conversion from sensor frame to world frame
	convertPose(sensor_pose, tf_sensor_wrt_world);				// apply sensor_pose parameters to the T matrix

	if( binParts_.count(cam_name) ) {	// For every new call, clear old values if present
			binParts_[cam_name].clear();
	}

	for( auto it = image_msg->models.begin(); it != image_msg->models.end(); ++it ) {
		geometry_msgs::Pose part_pose = it->pose;
		try{
			tf2::doTransform(part_pose, part_pose, tf_sensor_wrt_world);
		}
		catch (tf2::TransformException &ex) {
				ROS_WARN("exception while converting child frame pose to world frame");
		    	ROS_WARN("%s",ex.what());
		        ros::Duration(0.01).sleep();
		}
		auto part_type = it->type;
//		accumulated_binParts_[part_type].push_back(part_pose);
		binParts_[cam_name][part_type].push_back(part_pose);
	}
	// set flag of respective cameras once processed
	if( cam_name == "cam1" ) cam_1_processed_ = true;
	if( cam_name == "cam2" ) cam_2_processed_ = true;
}

void AriacSensorManager::beltlogicalCameraCallback( const osrf_gear::LogicalCameraImage::ConstPtr & image_msg ) {

	// when ordermanager finishes segregating orders
	if( orderManager->isOrderSegregated() ){
//		ROS_INFO_STREAM("Orders are Segregated, now let's pick up these parts" << std::endl);
		auto belt_order_parts = orderManager->getConveyorBeltOrderParts(); // get belt order parts

		auto sensor_pose = image_msg->pose;					// get sensor frame w.r.t world
		geometry_msgs::TransformStamped tf_sensor_wrt_world;	// T matrix for conversion from sensor frame to world frame

		convertPose(sensor_pose, tf_sensor_wrt_world);				// apply sensor_pose parameters to the T matrix

//		auto order = orderManager->getProductType();	// vector of part type in the order

		for(auto it = image_msg->models.begin(); it!=image_msg->models.end();++it) {

			for (auto order_it = belt_order_parts->begin(); order_it != belt_order_parts->end(); ++order_it) {

				if (tracking_part == nullptr && it->type.compare(order_it->first)) { // if camera part type == belt order type
					tracking_part = new osrf_gear::Model();
					tracking_part->type = it->type;
					tracking_part->pose = it->pose;
					geometry_msgs::Pose part_pose = it->pose;
					try{
						tf2::doTransform(part_pose, part_pose, tf_sensor_wrt_world);
					}
					catch (tf2::TransformException &ex) {
						ROS_WARN("exception while converting child frame pose to world frame");
						ROS_WARN("%s",ex.what());
						ros::Duration(0.01).sleep();
					}
					geometry_msgs::TransformStamped tf;
					convertPose(part_pose, tf);
					transform_publisher.publish(tf);
					//				ROS_INFO_STREAM("tracking part id: " << tracking_part->type << std::endl);
				}
			}

			if (it->type.compare(tracking_part->type) == 0 && it->pose.position.z > tracking_part->pose.position.z ) {
				convertPose(it->pose, tracking_part->pose);
				geometry_msgs::TransformStamped tf;
				convertPose(tracking_part->pose, tf);
				transform_publisher.publish(tf);
	//			ROS_INFO_STREAM("Tracking type: " << tracking_part->type << std::endl);
	//			ROS_INFO_STREAM("Tracking pose: \n" << tracking_part->pose << std::endl);
			}
		}

	}
}




bool AriacSensorManager::isObjectDetectedBreakBeam() {
	return object_detected;
}

void AriacSensorManager::breakBeamCallback(const osrf_gear::Proximity::ConstPtr & msg) {

	if (msg->object_detected) {  // If there is an object in proximity.
		ROS_INFO("Break beam triggered.");
		object_detected = true;
	}
	else{
		object_detected = false;
	}
}
