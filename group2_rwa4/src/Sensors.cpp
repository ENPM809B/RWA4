#include <Sensors.h>

AriacSensors::AriacSensors() {
  ROS_INFO("<<<<<< Subscribing to Logical Cameras >>>>>>");
  camSub1 = sensorNh.subscribe("/ariac/logical_camera_1", 10,
                                    &AriacSensors::LogicalCamera1Cb, this);
  camSub2 = sensorNh.subscribe("/ariac/logical_camera_2", 10,
                                    &AriacSensors::LogicalCamera2Cb, this);
  camSub3 = sensorNh.subscribe("/ariac/logical_camera_3", 10,
                                    &AriacSensors::LogicalCamera3Cb, this);
  cam3Count = 1;
  cam2Count = 1;
}

void AriacSensors::LogicalCamera1Cb(
             const osrf_gear::LogicalCameraImage::ConstPtr& image_msg) {
  // if (exit) return;

  // ROS_INFO_STREAM_THROTTLE(3, "Logical camera 1: '" <<
  //                          image_msg->models.size() << "' objects.");
  //
  // if (image_msg->models.size() != 0) {
  //   ROS_ERROR_STREAM("Logical Camera 1 does not see anything");
  // }

  cam1Parts = *image_msg;
}

void AriacSensors::LogicalCamera2Cb(
            const osrf_gear::LogicalCameraImage::ConstPtr& image_msg) {
  if (exit) return;

  ROS_INFO_STREAM_THROTTLE(3, "Logical camera 2: '" <<
                           image_msg->models.size() << "' objects.");

  if (image_msg->models.size() == 0) {
    ROS_ERROR_STREAM("Logical Camera 2 does not see anything");
  }

  cam2Parts = *image_msg;
}

void AriacSensors::LogicalCamera3Cb(
            const osrf_gear::LogicalCameraImage::ConstPtr& image_msg) {
  if (exit) return;

  ROS_INFO_STREAM_THROTTLE(3, "Logical camera 3: '" <<
                           image_msg->models.size() << "' objects.");

  if (image_msg->models.size() == 0) {
    ROS_ERROR_STREAM("Logical Camera 3 does not see anything");
  }

  cam3Parts = *image_msg;
}

void AriacSensors::CameraParts() {
  // std::string productFrame;
  // ROS_INFO_STREAM("cameraFrames initial size: " << cameraFrames.size());
  for(auto& part: cam3Parts.models) {
    std::string productFrame = "logical_camera_3_" + part.type + "_" +
                   std::to_string(cam3Count) + "_frame";
    cameraFrames[part.type].emplace_back(productFrame);
    cam3Count++;
  }
  // ROS_INFO_STREAM("cameraFrames size after adding Camera 3 parts: " << cameraFrames.size());
  for(auto& part: cam2Parts.models) {
    std::string productFrame = "logical_camera_2_" + part.type + "_" +
                     std::to_string(cam2Count) + "_frame";
    cameraFrames[part.type].emplace_back(productFrame);
    cam2Count++;
  }
  // ROS_INFO_STREAM("cameraFrames size after adding Camera 2 parts: " << cameraFrames+++.size());
  exit = true;
}

geometry_msgs::Pose AriacSensors::GetPose(const std::string& srcFrame, const std::string& targetFrame) {
  geometry_msgs::Pose partPose;
  ROS_INFO_STREAM("Getting part pose...");
  camTfListener.waitForTransform(srcFrame, cameraFrames[targetFrame].back(), ros::Time(0),
                                             ros::Duration(3));
  camTfListener.lookupTransform(srcFrame, cameraFrames[targetFrame].back(), ros::Time(0),
                                            camTfTransform);
  partPose.position.x = camTfTransform.getOrigin().getX();
  partPose.position.y = camTfTransform.getOrigin().getY();
  partPose.position.z = camTfTransform.getOrigin().getZ();
  partPose.orientation.x = camTfTransform.getRotation().getX();
  partPose.orientation.y = camTfTransform.getRotation().getY();
  partPose.orientation.z = camTfTransform.getRotation().getZ();
  partPose.orientation.w = camTfTransform.getRotation().getW();
  // ROS_INFO_STREAM_THROTTLE(3, "x: " << partPose.position.x <<
  //                             "y: " << partPose.position.y <<
  //                             "z: " << partPose.position.z);
  return partPose;
}

AriacSensors::~AriacSensors() {}
