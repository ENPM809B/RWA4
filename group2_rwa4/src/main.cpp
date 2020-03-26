#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <algorithm>
#include <vector>
#include <iostream>
#include <ros/ros.h>
#include <string>
#include <ctime>
#include <ros/service.h>
#include <osrf_gear/LogicalCameraImage.h>
#include <osrf_gear/Order.h>
#include <osrf_gear/Proximity.h>
#include <osrf_gear/VacuumGripperControl.h>
#include <osrf_gear/VacuumGripperState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>


class GetObject{
public:
  int object = 0;
  bool arm_engage = false;   
};

GetObject ObjectOnBelt;


std::vector<std::string> order_parts;
std::vector<std::string> belt_parts;
int counter{0};

int break_beam_counter = 0;

bool gripper_state_;
bool grab_now = false;

// Function Definitions
// -- Function 1: Start the competition by waiting for and then calling the start ROS Service.
void start_competition(ros::NodeHandle & node) {
  // Create a Service client for the correct service, i.e. '/ariac/start_competition'.
  ros::ServiceClient start_client = node.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
  // If it's not already ready, wait for it to be ready.
  // Calling the Service using the client before the server is ready would fail.
  if (!start_client.exists()) {
    ROS_INFO("Waiting for the competition to be ready...");
    start_client.waitForExistence();
    ROS_INFO("Competition is now ready.");
  }
  ROS_INFO("Requesting competition start...");
  std_srvs::Trigger srv;  // Combination of the "request" and the "response".
  start_client.call(srv);  // Call the start Service.
  if (!srv.response.success) {  // If not successful, print out why.
    ROS_ERROR_STREAM("Failed to start the competition: " << srv.response.message);
  }
  else {
    ROS_INFO("Competition started!");
  }
}

class Competition
{
public:
  explicit Competition(ros::NodeHandle & node)
  : current_score_(0), arm_1_has_been_zeroed_(false), arm_2_has_been_zeroed_(false)
  {
    // %Tag(ADV_CMD)%
//    arm_1_joint_trajectory_publisher_ = node.advertise<trajectory_msgs::JointTrajectory>(
//      "/ariac/arm1/arm/command", 10);
//
    arm_2_joint_trajectory_publisher_ = node.advertise<trajectory_msgs::JointTrajectory>(
      "/ariac/arm2/arm/command", 10);
//    // %EndTag(ADV_CMD)%
  }

  /// Called when a new message is received.
  void current_score_callback(const std_msgs::Float32::ConstPtr & msg) {
    if (msg->data != current_score_)
    {
      ROS_INFO_STREAM("Score: " << msg->data);
    }
    current_score_ = msg->data;
  }

  /// Called when a new message is received.
  void competition_state_callback(const std_msgs::String::ConstPtr & msg) {
    if (msg->data == "done" && competition_state_ != "done")
    {
      ROS_INFO("Competition ended.");
    }
    competition_state_ = msg->data;
  }

  /// Called when a new Order message is received.
  // -- Making Changes
  void order_callback(const osrf_gear::Order::ConstPtr & order_msg) {
    received_orders_.push_back(*order_msg);
    auto order_details = *order_msg ; 

    for (auto shipment :order_msg->shipments){
    	for (auto product:shipment.products){
    		// std::cout << "Part of Order:" << product.type << std::endl;
    		order_parts.push_back(product.type);
    	}
    }
  }

  /// Called when a new JointState message is received.
  void arm_1_joint_state_callback(
    const sensor_msgs::JointState::ConstPtr & joint_state_msg)
  {
    ROS_INFO_STREAM_THROTTLE(10,
      "Joint States arm 1 (throttled to 0.1 Hz):\n" << *joint_state_msg);
    // ROS_INFO_STREAM("Joint States:\n" << *joint_state_msg);
    arm_1_current_joint_states_ = *joint_state_msg;
    if (!arm_1_has_been_zeroed_) {
      arm_1_has_been_zeroed_ = true;
      ROS_INFO("Sending arm to zero joint positions...");
      send_arm_to_zero_state(arm_1_joint_trajectory_publisher_);
    }
  }

  void arm_2_joint_state_callback(
    const sensor_msgs::JointState::ConstPtr & joint_state_msg)
  {
    ROS_INFO_STREAM_THROTTLE(10,
      "Joint States arm 2 (throttled to 0.1 Hz):\n" << *joint_state_msg);
    // ROS_INFO_STREAM("Joint States:\n" << *joint_state_msg);
    arm_2_current_joint_states_ = *joint_state_msg;
    if (!arm_2_has_been_zeroed_) {
      arm_2_has_been_zeroed_ = true;
      ROS_INFO("Sending arm 2 to zero joint positions...");
      send_arm_to_zero_state(arm_2_joint_trajectory_publisher_);
    }
  }

  /// Create a JointTrajectory with all positions set to zero, and command the arm.
  void send_arm_to_zero_state(ros::Publisher & joint_trajectory_publisher) {
    // Create a message to send.
    trajectory_msgs::JointTrajectory msg;
    // Fill the names of the joints to be controlled.
    // Note that the vacuum_gripper_joint is not controllable.
    msg.joint_names.clear();
    msg.joint_names.push_back("shoulder_pan_joint");
    msg.joint_names.push_back("shoulder_lift_joint");
    msg.joint_names.push_back("elbow_joint");
    msg.joint_names.push_back("wrist_1_joint");
    msg.joint_names.push_back("wrist_2_joint");
    msg.joint_names.push_back("wrist_3_joint");
    msg.joint_names.push_back("linear_arm_actuator_joint");
    // Create one point in the trajectory.
    msg.points.resize(1);
    // Resize the vector to the same length as the joint names.
    // Values are initialized to 0.
    msg.points[0].positions.resize(msg.joint_names.size(), 0.0);
    // How long to take getting to the point (floating point seconds).
    msg.points[0].time_from_start = ros::Duration(0.001);
    ROS_INFO_STREAM("Sending command:\n" << msg);
    joint_trajectory_publisher.publish(msg);
  }

  void logical_camera_callback_1(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg){
    ROS_INFO_STREAM_THROTTLE(10,
      "Logical camera: '" << image_msg->models.size() << "' objects.");
  }

  void logical_camera_callback_4(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg)
  {
    ROS_INFO_STREAM_THROTTLE(10,
      "Logical camera: '" << image_msg->models.size() << "' objects.");
  }

	void logical_camera_callback_5(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg)
	{
  	auto imageMessage = *image_msg ;  	
  	if (imageMessage.models.size() != 0){
  	    if (belt_parts.size() == 0 || belt_parts.back() != imageMessage.models[0].type){
  	    	  belt_parts.push_back(imageMessage.models[0].type);
  	    	  counter++;
  	    	  if ((std::find(order_parts.begin(), order_parts.end(), imageMessage.models[0].type ) != order_parts.end())){
  	    	      if (!(ObjectOnBelt.arm_engage)){
  	    	          ObjectOnBelt.object = counter;
  	    	          ObjectOnBelt.arm_engage = true;
  	    	      }
  	    	  }
  	    }  	    
  	}
	}

  void break_beam_callback_1(const osrf_gear::Proximity::ConstPtr & msg) {
    if (msg->object_detected) {
      ROS_INFO("Break beam 1 triggered.");
    }
  }
  

private:
  std::string competition_state_;
  double current_score_;
  ros::Publisher arm_1_joint_trajectory_publisher_;
  ros::Publisher arm_2_joint_trajectory_publisher_;
  std::vector<osrf_gear::Order> received_orders_;
  sensor_msgs::JointState arm_1_current_joint_states_;
  sensor_msgs::JointState arm_2_current_joint_states_;
  bool arm_1_has_been_zeroed_;
  bool arm_2_has_been_zeroed_;
};

void proximity_sensor_callback(const sensor_msgs::Range::ConstPtr & msg) {
  if ((msg->max_range - msg->range) > 0.01) {  
    ROS_INFO_THROTTLE(1, "Proximity sensor sees something.");
  }
}

void laser_profiler_callback(const sensor_msgs::LaserScan::ConstPtr & msg) {
  size_t number_of_valid_ranges = std::count_if(
    msg->ranges.begin(), msg->ranges.end(), [](const float f) {return std::isfinite(f);});
  if (number_of_valid_ranges > 0) {
    ROS_INFO_THROTTLE(1, "Laser profiler sees something.");
  }
}

void break_beam_callback_2(const osrf_gear::Proximity::ConstPtr & msg) {
    if (msg->object_detected) {
        ROS_INFO("Break beam 2 triggered.");
        break_beam_counter += 1;
        grab_now = true;
     }
}

ros::ServiceClient gripper_client_;


void GripperCallback(
        const osrf_gear::VacuumGripperState::ConstPtr& grip) {
    gripper_state_ = grip->attached;
}


void GripperToggle(const bool& state) {
    osrf_gear::VacuumGripperControl gripper_service_;
    gripper_service_.request.enable = state;
    gripper_client_.call(gripper_service_);
    ros::Duration(1.0).sleep();
    if (gripper_service_.response.success) {
        ROS_INFO_STREAM("Gripper activated!");
    } else {
        ROS_WARN_STREAM("Gripper activation failed!");
    }
}


int main(int argc, char ** argv) {
  ros::init(argc, argv, "kitting"); 
  ros::NodeHandle node;
  ros::NodeHandle node_("/ariac/arm1");
  ros::NodeHandle gripper_nh_;
  geometry_msgs::Pose final;
  geometry_msgs::Pose temp_pose1;
  geometry_msgs::Pose grab_pose;
  geometry_msgs::Pose place_pose;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit::planning_interface::MoveGroupInterface::Options loadOptions("manipulator","/ariac/arm1/robot_description",node_);
  moveit::planning_interface::MoveGroupInterface robot_move_group(loadOptions);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface::Plan planner_;


  Competition comp_class(node);
  ros::Subscriber current_score_subscriber = node.subscribe("/ariac/current_score", 10,&Competition::current_score_callback, &comp_class);
  ros::Subscriber competition_state_subscriber = node.subscribe("/ariac/competition_state", 10,&Competition::competition_state_callback, &comp_class);
  ros::Subscriber orders_subscriber = node.subscribe("/ariac/orders", 10,&Competition::order_callback, &comp_class);
  ros::Subscriber proximity_sensor_subscriber = node.subscribe("/ariac/proximity_sensor_1", 10, proximity_sensor_callback);
  ros::Subscriber break_beam_subscriber_1 = node.subscribe("/ariac/break_beam_1_change", 10, &Competition::break_beam_callback_1, &comp_class);
  ros::Subscriber logical_camera_subscriber_1 = node.subscribe("/ariac/logical_camera_1", 10, &Competition::logical_camera_callback_1, &comp_class);
  ros::Subscriber logical_camera_subscriber_4 = node.subscribe("/ariac/logical_camera_4", 10, &Competition::logical_camera_callback_4, &comp_class);
  ros::Subscriber logical_camera_subscriber_5 = node.subscribe("/ariac/logical_camera_5", 10, &Competition::logical_camera_callback_5, &comp_class);
  ros::Subscriber laser_profiler_subscriber = node.subscribe("/ariac/laser_profiler_1", 10, laser_profiler_callback);
  ros::Subscriber gripper_subscriber_;
  gripper_subscriber_ = gripper_nh_.subscribe("/ariac/arm1/gripper/state", 10, GripperCallback);
  gripper_client_ = node_.serviceClient<osrf_gear::VacuumGripperControl>("/ariac/arm1/gripper/control");
  ros::Subscriber break_beam_subscriber_2 = node_.subscribe("/ariac/break_beam_2_change", 10,   break_beam_callback_2);
  ROS_INFO("Setup complete.");
    
  robot_move_group.setPlanningTime(20);
  robot_move_group.setNumPlanningAttempts(10);
  robot_move_group.setPlannerId("RRTConnectkConfigDefault");
  robot_move_group.setMaxVelocityScalingFactor(0.9);
  robot_move_group.setMaxAccelerationScalingFactor(0.9);
  robot_move_group.allowReplanning(true);
  start_competition(node);

  std::vector<double> home;


  //horizontal position above the conveyor
  //home = {0.0, 0, 0, 0, 0, 0, 0};
  // robot_move_group.setJointValueTarget(home);

  //V position above conveyor
  std::map<std::string, double> temp_pose_;
  temp_pose_["shoulder_pan_joint"] = 0;
  temp_pose_["shoulder_lift_joint"] = -0.5;
  temp_pose_["elbow_joint"] = 0.5;
  temp_pose_["wrist_1_joint"] = 0;
  temp_pose_["wrist_2_joint"] = 0;
  temp_pose_["wrist_3_joint"] = 0;
  temp_pose_["linear_arm_actuator_joint"] = 0;

  robot_move_group.setJointValueTarget(temp_pose_);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan1;
  robot_move_group.move();
  ros::Duration(0.5).sleep();

  //pose near the conveyor belt for grabbing
  final.orientation.w = 0.707;
  final.orientation.y = 0.707;
  final.position.x = 1.22;
  final.position.y = 0.7;
  final.position.z = 0.95;

  robot_move_group.setPoseTarget(final);
  robot_move_group.move();

  grab_pose = final;
  place_pose = final;
  grab_pose.position.z = 0.928;

  while (ros::ok()){
        if(grab_now == true && ObjectOnBelt.object==break_beam_counter && ObjectOnBelt.arm_engage == true){
            robot_move_group.setPoseTarget(grab_pose);
            moveit::planning_interface::MoveGroupInterface::Plan grab_plan;
            robot_move_group.move();
            GripperToggle(true);
            grab_now = false;
            ObjectOnBelt.arm_engage = false;            
            if (gripper_state_){
                place_pose.position.x = 0.25;
                place_pose.position.y = 1;
        				place_pose.position.z = 1.2;
        				robot_move_group.setPoseTarget(place_pose);
        				moveit::planning_interface::MoveGroupInterface::Plan throw_plan;
                robot_move_group.move();			

                robot_move_group.setPoseTarget(place_pose);

                robot_move_group.move();
                ros::Duration(0.5).sleep();
              }                
        }
  }
  ros::spin(); 
  return 0;
}

