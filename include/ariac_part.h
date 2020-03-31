#pragma once

#include <string>
#include <geometry_msgs/Pose.h>

class AriacOrderPart {
private:
	std::string part_type_;
	geometry_msgs::Pose end_pose_;
	geometry_msgs::Pose current_pose_;
	std::string agv_id_;

public:
	AriacOrderPart();
	AriacOrderPart(std::string, std::string, geometry_msgs::Pose);
	~AriacOrderPart();

	// Setters
	void setPartType(std::string);
	void setPartEndPose(geometry_msgs::Pose);
	void setPartCurrentPose(geometry_msgs::Pose);
	void setPartAgvID(std::string);

	// Getters
	std::string getPartType() const;
	geometry_msgs::Pose getPartEndPose() const;
	geometry_msgs::Pose getPartCurrentPose() const;
	std::string getPartAgvID() const;
//	friend std::ostream& operator << (std::ostream& os, const Order& order); // for printing orderparts
};
