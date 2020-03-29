#include <ros/ros.h>
#include <Competition.h>
#include <Orders.h>
#include <RobotController.h>
#include <Sensors.h>

int main(int argc, char **argv) {
    ROS_INFO("Starting main function");
    ros::init(argc, argv, "RWA4_node");
    Competition comp;
    // RobotControl("arm1") arm1;
    // AriacSensors camera;
    AriacOrders order;
    ros::NodeHandle node;

    comp.startCompetition(node);

    ROS_INFO("Setup complete.");

    ros::Duration(2.0).sleep();

    order.readOrder();

    // arm1.SendRobotHome();
    // while (ros::ok())
    // camera.CameraParts();

    // geometry_msgs::Pose pose = camera.GetPose("/world", "gasket_part");
    // arm1.GoToTarget(pose);
    ros::spin();

    return 0;
}
