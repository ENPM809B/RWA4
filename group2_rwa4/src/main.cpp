#include <ros/ros.h>
#include <Competition.h>
#include <Orders.h>
#include <RobotController.h>
#include <Sensors.h>

int main(int argc, char **argv) {
    ROS_INFO("Starting main function");
    ros::init(argc, argv, "RWA4_node");
    Competition comp;
    AriacSensors camera;
    AriacOrders order;
    RobotControl arm1;
    ros::NodeHandle node;

    comp.startCompetition(node);

    ROS_INFO("Setup complete.");

    ros::Duration(2.0).sleep();

    order.readOrder();

    // arm1.SendRobotHome();

    camera.CameraParts();

    geometry_msgs::Pose pose = camera.GetPose("/world", "gasket_part");
    arm1.GoToTarget(pose);
    ros::spin();

    return 0;
}
