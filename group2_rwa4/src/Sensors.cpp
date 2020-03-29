#include <Sensors.h>

AriacSensors::AriacSensors() {
  ROS_INFO("<<<<<< Subscribing to Logical Cameras >>>>>>");
  camSub1 = sensorNh.subscribe("/ariac/logical_camera_1", 10,
                                    &AriacSensors::LogicalCamera1Cb, this);
  camSub2 = sensorNh.subscribe("/ariac/logical_camera_2", 10,
                                    &AriacSensors::LogicalCamera2Cb, this);
  camSub3 = sensorNh.subscribe("/ariac/logical_camera_3", 10,
                                    &AriacSensors::LogicalCamera3Cb, this);
  cam1Count = 0;
  cam2Count = 1;
  cam3Count = 1;
}

void AriacSensors::LogicalCamera1Cb(
             const osrf_gear::LogicalCameraImage::ConstPtr& image_msg) {

  ROS_INFO_STREAM_THROTTLE(3, "Logical camera 1: '" <<
                           image_msg->models.size() << "' objects.");

  // if (image_msg->models.size() == 0) {
  //   ROS_ERROR_STREAM("Logical Camera 1 does not see anything");
  // }

  cam1Parts = *image_msg;

  if(cam1Parts.models.size() != 0) {
    CameraParts(1);
  }
}

void AriacSensors::LogicalCamera2Cb(
            const osrf_gear::LogicalCameraImage::ConstPtr& image_msg) {
  // if (exit2) return;

  ROS_INFO_STREAM_THROTTLE(3, "Logical camera 2: '" <<
                           image_msg->models.size() << "' objects.");

  if (image_msg->models.size() == 0) {
    ROS_ERROR_STREAM("Logical Camera 2 does not see anything");
  }

  cam2Parts = *image_msg;

  if(cam2Parts.models.size() != 0) {
    CameraParts(2);
  }
}

void AriacSensors::LogicalCamera3Cb(
            const osrf_gear::LogicalCameraImage::ConstPtr& image_msg) {
  // if (exit3) return;

  ROS_INFO_STREAM_THROTTLE(3, "Logical camera 3: '" <<
                           image_msg->models.size() << "' objects.");

  if (image_msg->models.size() == 0) {
    ROS_ERROR_STREAM("Logical Camera 3 does not see anything");
  }

  cam3Parts = *image_msg;
  if(cam3Parts.models.size() != 0) {
    CameraParts(3);
  }
}

void AriacSensors::CameraParts(int id) {
  if (id == 1) {
    for(auto& part: cam1Parts.models) {
      std::string productFrame = "logical_camera_1_" + part.type + "_" +
                     std::to_string(cam1Count) + "_frame";
      cameraFrames[part.type].emplace_back(productFrame);
      cam1Count++;
    }
  }

  if (id == 2 && exit2 == false) {
    for(auto& part: cam2Parts.models) {
      std::string productFrame = "logical_camera_2_" + part.type + "_" +
                       std::to_string(cam2Count) + "_frame";
      cameraFrames[part.type].emplace_back(productFrame);
      cam2Count++;
    }
    exit2 = true;
  }

  if (id == 3 && exit3 == false) {
    for(auto& part: cam3Parts.models) {
      std::string productFrame = "logical_camera_3_" + part.type + "_" +
                         std::to_string(cam3Count) + "_frame";
      cameraFrames[part.type].emplace_back(productFrame);
      cam3Count++;
    }
    exit3 = true;
  }

}

geometry_msgs::Pose AriacSensors::GetPose(const std::string& srcFrame, const std::string& targetFrame) {
  geometry_msgs::Pose partPose;
  ROS_INFO_STREAM("Getting part pose...");
  camTfListener.waitForTransform(srcFrame, targetFrame, ros::Time(0),
                                             ros::Duration(3));
  camTfListener.lookupTransform(srcFrame, targetFrame, ros::Time(0),
                                            camTfTransform);
  partPose.position.x = camTfTransform.getOrigin().getX();
  partPose.position.y = camTfTransform.getOrigin().getY();
  partPose.position.z = camTfTransform.getOrigin().getZ();

  // arm1_.GoToTarget(partPose);
  // arm1_.SendRobotHome();
  return partPose;
}

AriacSensors::~AriacSensors() {}
