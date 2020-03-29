#ifndef INCLUDE_SENSORS_H
#define INCLUDE_SENSORS_H

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <osrf_gear/LogicalCameraImage.h>
#include <RobotController.h>
#include <iostream>
#include <string>
#include <map>
#include <vector>

class AriacSensors {
private:
  //
  ros::NodeHandle sensorNh;
  //
  ros::Subscriber camSub1;
  //
  ros::Subscriber camSub2;
  //
  ros::Subscriber camSub3;
  //
  tf::TransformListener camTfListener;
  //
  tf::StampedTransform camTfTransform;
  //
  osrf_gear::LogicalCameraImage cam1Parts;
  //
  osrf_gear::LogicalCameraImage cam2Parts;
  //
  osrf_gear::LogicalCameraImage cam3Parts;
  //
  int cam1Count, cam2Count, cam3Count;
  //
  // std::map<std::string, std::vector<std::string>> cameraFrames;

  // RobotControl arm1_;
  //
  bool exit2{false}, exit3{false};

public:
  std::map<std::string, std::vector<std::string>> cameraFrames;

  AriacSensors();

  void LogicalCamera1Cb(const osrf_gear::LogicalCameraImage::ConstPtr &);

  void LogicalCamera2Cb(const osrf_gear::LogicalCameraImage::ConstPtr &);

  void LogicalCamera3Cb(const osrf_gear::LogicalCameraImage::ConstPtr &);

  void CameraParts(int id);

  geometry_msgs::Pose GetPose(const std::string& srcFrame, const std::string& targetFram);

  // geometry_msgs::Pose GetPose(const std::string& srcFrame);

  ~AriacSensors();

};

#endif // INCLUDE_SENSORS_H
