#include <Orders.h>

/*
 * @brief
 *
 * @param
 *
 * @return
 */
AriacOrders::AriacOrders(): arm1_{"arm1"} {
  //
  order_subscriber_ = order_manager_nh_.subscribe("/ariac/orders", 10, &AriacOrders::orderCb, this);
}

/*
 * @brief
 *
 * @param
 *
 * @return
 */
void AriacOrders::orderCb(const osrf_gear::Order::ConstPtr& order_msg) {
  received_orders_.push_back(*order_msg);
}

/*
 * @brief
 *
 * @param
 *
 * @return
 */
void AriacOrders::readOrder() {
  ROS_WARN("<<<<< Reading Order >>>>>");
  ros::spinOnce();
  ros::Duration(1.0).sleep();
  for (const auto &order : received_orders_){
    auto order_id = order.order_id;
    auto shipments = order.shipments;
    for (const auto &shipment: shipments){
      auto shipment_type = shipment.shipment_type;
      auto agv = shipment.agv_id.back();//--this returns a char
      int agv_id = (shipment.agv_id == "any") ? 1 : agv - '0';
      auto products = shipment.products;
      ROS_INFO_STREAM("Received Order");
      ROS_INFO_STREAM("Order ID: " << order_id);
      ROS_INFO_STREAM("Shipment Type: " << shipment_type);
      ROS_INFO_STREAM("AGV ID: " << agv_id);
      for (const auto &product: products){
        ros::spinOnce();
        if (camera.cameraFrames.find(product.type) != camera.cameraFrames.end()) {
          product_type_pose_.first = product.type;
          ROS_INFO_STREAM("Product type: " << product_type_pose_.first);
          product_type_pose_.second = product.pose;
          ROS_INFO_STREAM("Product pose : " << orderParts[product.type]);
          std::string frame = camera.cameraFrames[product.type].back();
          ROS_INFO_STREAM(frame);
          geometry_msgs::Pose partPose = camera.GetPose("/world", frame);
          arm1_.GoToTarget(partPose);
          partPose.position.z += 0.2;
          arm1_.GoToTarget(partPose);
          ROS_INFO_STREAM("Actuating the gripper...");
          arm1_.GripperToggle(true);
          arm1_.SendRobotHome();
          arm1_.GripperToggle(false);
          // camera.cameraFrames[product.type].pop_back();
        }
      }
      break;
    }
  }
}

/*
 * @brief
 *
 * @param
 *
 * @return
 */
AriacOrders::~AriacOrders() {};
