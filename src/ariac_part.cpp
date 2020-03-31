//
// Created by zeid on 3/1/20.
//

#include <ariac_part.h>

AriacPart::AriacPart()
{}

AriacPart::AriacPart(std::string type, std::string agv_id, geometry_msgs::Pose pose): part_type_(type), agv_id_(agv_id), end_pose_(pose)
{}

AriacPart::~AriacPart()
{}

void AriacPart::setPartType(std::string part_type) {
    part_type_ = part_type;
}

void AriacPart::setPartEndPose(geometry_msgs::Pose part_pose) {
    end_pose_ = part_pose;
//    end_pose_.position.z +=0.3;
}

void AriacPart::setPartCurrentPose(geometry_msgs::Pose pose) {
    current_pose_ = pose;
}

void AriacPart::setPartAgvID(std::string id){
	agv_id_ = id;
}

std::string AriacPart::getPartType() const {
    return part_type_;
}

geometry_msgs::Pose AriacPart::getPartEndPose() const {
    return end_pose_;
}

geometry_msgs::Pose AriacPart::getPartCurrentPose() const {
    return current_pose_;
}

std::string AriacPart::getPartAgvID() const {
	return agv_id_;
}

//std::ostream& operator << (std::ostream& os, const AriacPart& orderPart) {
//	os << "shipment_type : " << orderPart.shipment_type << "\n";
//	os << "agv_id : " << orderPart.agv_id << "\n";
//	for(auto it = orderPart.products.begin(); it!=orderPart.products.end(); ++it){
//		os << "product type : " << it->first << "\n";
//		os << "product pose : " << it->second << "\n";
//	}
//	return os;
//}
