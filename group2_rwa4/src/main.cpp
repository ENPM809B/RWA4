#include <string>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <algorithm>
#include <vector>
#include <iostream>
#include <ros/ros.h>

#include <ctime>
#include <ros/service.h>
#include <osrf_gear/LogicalCameraImage.h>
#include <osrf_gear/Order.h>
#include <osrf_gear/Proximity.h>
#include <osrf_gear/VacuumGripperControl.h>
#include <osrf_gear/VacuumGripperState.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

using namespace std;

class GetObject{
public:
  int object = 0;
  bool arm1_engage = false;   
};

GetObject ObjectOnBelt;


std::vector<std::string> order_parts;
std::vector<std::string> belt_parts;
int counter{0};

int break_beam_counter = 0;

bool gripper_state_1;
bool grab_now_1 = false;

// std::vector<float> pose;


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
  float pose[6];
  float roll_grab, pitch_grab, yaw_grab;
  float x_grab, y_grab, z_grab;
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

  void logical_camera_callback_4(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg)
  {
    ROS_INFO_STREAM_THROTTLE(10,
      "Logical camera: '" << image_msg->models.size() << "' objects.");
  }

	/*void logical_camera_callback_5(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg)
	{
  	auto imageMessage = *image_msg ;  
    int obj_count = 1;	
    ros::Duration timeout(5.0);
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    
  	if (imageMessage.models.size() != 0){
  	    if (belt_parts.size() == 0 || belt_parts.back() != imageMessage.models[0].type){
  	    	  belt_parts.push_back(imageMessage.models[0].type);
  	    	  counter++;
  	    	  if ((std::find(order_parts.begin(), order_parts.end(), imageMessage.models[0].type ) != order_parts.end())){
  	    	      if (!(ObjectOnBelt.arm1_engage)){
  	    	          ObjectOnBelt.object = counter;
  	    	          ObjectOnBelt.arm1_engage = true;
  	    	      }
  	    	  }
  	    }
          	    
  	}    
	}*/

  
  void logical_camera_callback_5(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg)
  {
    auto imageMessage = *image_msg ;  
    int obj_count = 1;  
    // ros::Duration timeout(1.0);
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);     
    while(obj_count <= image_msg->models.size()){
      geometry_msgs::TransformStamped tf_camera_wrt_world;
      //piston rod part is obtained by rostpic echo /ariac/logical_camera_4
      string cam_frame = "logical_camera_4_piston_rod_part_" + to_string(obj_count) + "_frame";
      try{
          // transformStamped = tfBuffer.lookupTransform("world", 
          //   "logical_camera_4_piston_rod_part_" + to_string(obj_count) + "_frame",
          //  ros::Time(0));
            // transformStamped = tfBuffer.lookupTransform("world", cam_frame, ros::Time(0));
        tf_camera_wrt_world.transform.translation.x = image_msg->pose.position.x;
        tf_camera_wrt_world.transform.translation.y = image_msg->pose.position.y;
        tf_camera_wrt_world.transform.translation.z = image_msg->pose.position.z;
        tf_camera_wrt_world.transform.rotation.w = image_msg->pose.orientation.w;
        tf_camera_wrt_world.transform.rotation.x = image_msg->pose.orientation.x;
        tf_camera_wrt_world.transform.rotation.y = image_msg->pose.orientation.y;
        tf_camera_wrt_world.transform.rotation.z = image_msg->pose.orientation.z;

        for(auto it=image_msg->models.begin(); it<image_msg->models.end(); ++it){
//          std::cout << ".............wrt camera frame...............\n" << it->pose;

          geometry_msgs::Pose t_pose = it->pose;
          tf2::Quaternion q(t_pose.orientation.x,t_pose.orientation.y,t_pose.orientation.z,t_pose.orientation.w);
            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
          m.getRPY(roll, pitch, yaw);
          // ROS_INFO("object in camera frame : [%f,%f,%f] [%f,%f,%f]", t_pose.position.x,
          //     t_pose.position.y, t_pose.position.z, roll, pitch, yaw);
          tf2::doTransform(t_pose, t_pose, tf_camera_wrt_world);
            q = tf2::Quaternion(t_pose.orientation.x,t_pose.orientation.y,t_pose.orientation.z,t_pose.orientation.w);
            m = tf2::Matrix3x3(q);
            m.getRPY(roll, pitch, yaw);
//          std::cout << "..............wrt world frame...............\n" << t_pose << "  roll: "<< roll << "\n  pitch: "<< pitch << "\n  yaw: " << yaw << "\n\n";
            // ROS_INFO("object in world frame : [%f,%f,%f] [%f,%f,%f]", t_pose.position.x,
            //               t_pose.position.y, t_pose.position.z, roll, pitch, yaw);
            roll_grab = roll;
            pitch_grab = pitch;
            yaw_grab = yaw;
            x_grab = t_pose.position.x;
            y_grab = t_pose.position.y;
            z_grab = t_pose.position.z;
        }
      }
      catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        ros::Duration(1.0).sleep();
        continue;
      }
      
      
      if (belt_parts.size() == 0 || belt_parts.back() != imageMessage.models[0].type){
      belt_parts.push_back(imageMessage.models[0].type);
      counter++;
      if ((std::find(order_parts.begin(), order_parts.end(), imageMessage.models[0].type ) != order_parts.end())){
          if (!(ObjectOnBelt.arm1_engage)){
              ObjectOnBelt.object = counter;
              ObjectOnBelt.arm1_engage = true;
          }
          cout << ObjectOnBelt.object << endl;
          cout << ObjectOnBelt.arm1_engage << endl;
          cout <<break_beam_counter << endl;
        }
      }

      

      // ROS_INFO("piston_rod_part_%d in world frame: [%f,%f,%f] [%f,%f,%f]", obj_count, transformStamped.transform.translation.x,
      // transformStamped.transform.translation.y, transformStamped.transform.translation.z, roll, pitch, yaw);      
      obj_count+=1;
      
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
        grab_now_1 = true;
     }
}

ros::ServiceClient gripper_client_;
ros::ServiceClient gripper_client_2;


void GripperCallback(
        const osrf_gear::VacuumGripperState::ConstPtr& grip) {
    gripper_state_1 = grip->attached;
    // ROS_INFO_STREAM(gripper_state_1);
}


void GripperToggle(const bool& state) {
    osrf_gear::VacuumGripperControl gripper_service_;
    gripper_service_.request.enable = state;
    gripper_client_.call(gripper_service_);
    // ros::Duration(1.0).sleep();
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
  // ros::NodeHandle node_2("/ariac/arm2");
  ros::NodeHandle gripper_nh_;
  // ros::NodeHandle gripper_nh_2;
  geometry_msgs::Pose final;
  // geometry_msgs::Pose final_2;
  geometry_msgs::Pose temp_pose1;
  geometry_msgs::Pose grab_pose;
  geometry_msgs::Pose place_pose;

  int return_counter = 0;

  //move() is blocking, requires asyn spinner
  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit::planning_interface::MoveGroupInterface::Options loadOptions("manipulator","/ariac/arm1/robot_description",node_);
  moveit::planning_interface::MoveGroupInterface robot_move_group(loadOptions);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface::Plan planner_;

  // moveit::planning_interface::MoveGroupInterface::Options loadOptions_2("manipulator","/ariac/arm2/robot_description",node_2);
  // moveit::planning_interface::MoveGroupInterface robot_move_group_2(loadOptions_2);
  // moveit::planning_interface::PlanningSceneInterface planning_scene_interface_2;
  // moveit::planning_interface::MoveGroupInterface::Plan planner_2;


  Competition comp_class(node);
  ros::Subscriber current_score_subscriber = node.subscribe("/ariac/current_score", 10,&Competition::current_score_callback, &comp_class);
  ros::Subscriber competition_state_subscriber = node.subscribe("/ariac/competition_state", 10,&Competition::competition_state_callback, &comp_class);
  ros::Subscriber orders_subscriber = node.subscribe("/ariac/orders", 10,&Competition::order_callback, &comp_class);
  // ros::Subscriber proximity_sensor_subscriber = node.subscribe("/ariac/proximity_sensor_1", 10, proximity_sensor_callback);
  // ros::Subscriber break_beam_subscriber_1 = node.subscribe("/ariac/break_beam_1_change", 10, &Competition::break_beam_callback_1, &comp_class);
  // ros::Subscriber logical_camera_subscriber_4 = node.subscribe("/ariac/logical_camera_4", 10, &Competition::logical_camera_callback_4, &comp_class);
  ros::Subscriber logical_camera_subscriber_5 = node.subscribe("/ariac/logical_camera_5", 10, &Competition::logical_camera_callback_5, &comp_class);
  // ros::Subscriber laser_profiler_subscriber = node.subscribe("/ariac/laser_profiler_1", 10, laser_profiler_callback);
  ros::Subscriber gripper_subscriber_;
  ros::Subscriber gripper_subscriber_2;

  gripper_subscriber_ = gripper_nh_.subscribe("/ariac/arm1/gripper/state", 10, GripperCallback);
  // gripper_subscriber_2 = gripper_nh_2.subscribe("/ariac/arm2/gripper/state", 10, GripperCallback);

  gripper_client_ = node_.serviceClient<osrf_gear::VacuumGripperControl>("/ariac/arm1/gripper/control");
  // gripper_client_2 = node_.serviceClient<osrf_gear::VacuumGripperControl>("/ariac/arm2/gripper/control");

  ros::Subscriber break_beam_subscriber_2 = node_.subscribe("/ariac/break_beam_2_change", 10, break_beam_callback_2);
  ROS_INFO("Setup complete.");
    
  robot_move_group.setPlanningTime(20);
  robot_move_group.setNumPlanningAttempts(10);
  robot_move_group.setPlannerId("RRTConnectkConfigDefault");
  robot_move_group.setMaxVelocityScalingFactor(0.9);
  robot_move_group.setMaxAccelerationScalingFactor(0.9);
  robot_move_group.allowReplanning(true);

  // robot_move_group_2.setPlanningTime(20);
  // robot_move_group_2.setNumPlanningAttempts(10);
  // robot_move_group_2.setPlannerId("RRTConnectkConfigDefault");
  // robot_move_group_2.setMaxVelocityScalingFactor(0.9);
  // robot_move_group_2.setMaxAccelerationScalingFactor(0.9);
  // robot_move_group_2.allowReplanning(true);

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
  // robot_move_group_2.setJointValueTarget(temp_pose_);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan1;
  // moveit::planning_interface::MoveGroupInterface::Plan my_plan2;

  robot_move_group.move();
  // robot_move_group_2.move();
  ros::Duration(0.5).sleep();

  

  //pose near the conveyor belt for grabbing
  final.orientation.w = 0.707;
  final.orientation.y = 0.707;
  final.position.x = 1.22;
  final.position.y = 1.9; //0.7 - 1.8
  final.position.z = 0.95;


  robot_move_group.setPoseTarget(final);
  robot_move_group.move();

  // robot_move_group_2.setPoseTarget(final_2);
  // robot_move_group_2.move();

  grab_pose = final;
  place_pose = final;
  // grab_pose.
  // grab_pose.position.x = comp_class.x_grab;
  // ROS_INFO("X: %f", comp_class.x_grab);
  // grab_pose.position.y = comp_class.y_grab;
  // ROS_INFO("Y: %f", comp_class.y_grab);
  grab_pose.position.z = 0.928; //928



  while (ros::ok()){
        /*if(return_counter > 0){
          // grab_pose = final;
          ROS_INFO("Earlier grab position achieved!");
          robot_move_group.setPoseTarget(final);
          robot_move_group.move();
          break
        }*/
        // ros::Subscriber logical_camera_subscriber_5 = node.subscribe("/ariac/logical_camera_5", 10, &Competition::logical_camera_callback_5, &comp_class);
        // grab_pose.position.y = comp_class.y_grab;
        ROS_INFO("X pos: %d",ObjectOnBelt.object);
        ROS_INFO("Y pos: %d",break_beam_counter);
        if(grab_now_1 == true && ObjectOnBelt.object==break_beam_counter && ObjectOnBelt.arm1_engage == true){
            //delay added to avoid multiple break beam detection
            ros::Duration(1.0).sleep();
            ros::Subscriber logical_camera_subscriber_5 = node.subscribe("/ariac/logical_camera_5", 10, 
              &Competition::logical_camera_callback_5, &comp_class);
            grab_pose.position.x = comp_class.x_grab;
            grab_pose.position.y = comp_class.y_grab-0.15;
            // ROS_INFO("X pos: %d",ObjectOnBelt.object);
            // ROS_INFO("Y pos: %d",break_beam_counter);
            robot_move_group.setPoseTarget(grab_pose);
            GripperToggle(true);
            moveit::planning_interface::MoveGroupInterface::Plan grab_plan;
            robot_move_group.move();
            
            grab_now_1 = false;
            ObjectOnBelt.arm1_engage = false; 
            // ros::Duration(5.0).sleep();  
            ROS_INFO("Gripper state: %d", gripper_state_1);    
            if (gripper_state_1){
                ROS_INFO_STREAM("In if");
                place_pose.position.x = 0.25;
                place_pose.position.y = 1;
        				place_pose.position.z = 1.2;
        				robot_move_group.setPoseTarget(place_pose);
        				moveit::planning_interface::MoveGroupInterface::Plan throw_plan;
                robot_move_group.move();			

                robot_move_group.setPoseTarget(place_pose);

                robot_move_group.move();
                ros::Duration(0.5).sleep();
                GripperToggle(false);
                // return_counter++;
              } 
              robot_move_group.setPoseTarget(final);
              robot_move_group.move();

        }
  }
  ros::spin(); 
  return 0;
}

