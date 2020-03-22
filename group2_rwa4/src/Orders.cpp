#include <Orders.h>

/*
 * @brief
 *
 * @param
 *
 * @return
 */
AriacOrders::AriacOrders() {
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
        product_type_pose_.first = product.type;
        ROS_INFO_STREAM("Product type: " << product_type_pose_.first);
        product_type_pose_.second = product.pose;
        ROS_INFO_STREAM("Product pose : " << product_type_pose_.second);
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
