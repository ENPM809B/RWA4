#ifndef INCLUDE_ORDERS_H
#define INCLUDE_ORDERS_H

#include <ros/ros.h>
#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <osrf_gear/Order.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <std_srvs/Trigger.h>
#include <Sensors.h>
#include <RobotController.h>

class AriacOrders {
private:
  //
  std::vector<osrf_gear::Order> received_orders_;
  //
  ros::NodeHandle order_manager_nh_;
  //
  ros::Subscriber order_subscriber_;
  //
  std::pair<std::string,geometry_msgs::Pose> product_type_pose_;
  //
  std::map<std::string, geometry_msgs::Pose> orderParts;
  //
  AriacSensors camera;
  //
  RobotControl arm1_;

public:
 /*
  * @brief
  *
  * @param
  *
  * @return
  */
  AriacOrders();

 /*
  * @brief
  *
  * @param
  *
  * @return
  */
  void orderCb(const osrf_gear::Order::ConstPtr& order_msg);

 /*
  * @brief
  *
  * @param
  *
  * @return
  */
  void readOrder();

 /*
  * @brief
  *
  * @param
  *
  * @return
  */
  ~AriacOrders();

};

#endif // INCLUDE_ORDERS_H
