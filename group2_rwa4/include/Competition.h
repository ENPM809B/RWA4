#ifndef INCLUDE_COMPETITION_H
#define INCLUDE_COMPETITION_H

#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <osrf_gear/LogicalCameraImage.h>

class Competition {
public:
 /*
  * @brief
  *
  * @param
  *
  * @return
  */
  Competition();

 /*
  * @brief
  *
  * @param
  *
  * @return
  */
  void startCompetition(ros::NodeHandle & node);

 /*
  * @brief
  *
  * @param
  *
  * @return
  */
  void endCompetition(ros::NodeHandle & node);

  void logical_camera_callback(
    const osrf_gear::LogicalCameraImage::ConstPtr & image_msg);

 /*
  * @brief
  *
  * @param
  *
  * @return
  */
  ~Competition();
};

#endif //INCLUDE_COMPETITION_H
