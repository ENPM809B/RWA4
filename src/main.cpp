#include <osrf_gear/AGVControl.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <tf/tf.h>
#include <iostream>
#include "order_manager.h"
#include "../include/competition.h"
#include "competition.h"
#include "../include/sensor.h"

int main(int argc, char **argv) {

    ROS_INFO("Starting main function");
    ros::init(argc, argv, "ariac_manager_node"); // initialize the node

    // Multi-threading
    ros::AsyncSpinner async_spinner(4);
    async_spinner.start();

    ros::NodeHandle node;
    AriacOrderManager manager(&node);
    AriacSensorManager sense(&manager);

    ROS_INFO("Setup complete.");

    Competition comp; // creates a Competition object and starts it

//    ros::Rate(10.0).sleep();
//    ros::Duration(2.0).sleep();
    //manager.SetScannedProducts();
//    manager.ExecuteOrder();

//    ros::spin();  // This executes callbacks on new data until ctrl-c.
    ros::waitForShutdown();

    //manager.ExecuteOrder();
    //EndCompetition(node);

    // ROS_WARN_STREAM("Killing the node....");

    return 0;
}
