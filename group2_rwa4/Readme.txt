Launch file
-> roslaunch group2_rwa4 group2-rwa4.launch

ROS node
-> rosrun group2_rwa4 kit_rwa4

Moveit
-> roslaunch ur10_moveit_config move_group.launch arm_namespace:=/ariac/arm1

RQT
-> rqt robot_description:=/ariac/arm1/robot_description
