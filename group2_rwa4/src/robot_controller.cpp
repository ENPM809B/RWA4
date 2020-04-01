//
// Created by zeid on 2/27/20.
//
#include "robot_controller.h"

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <moveit/robot_model_loader/robot_model_loader.h>



/**
 * Constructor for the robot
 * Class attributes are initialized in the constructor init list
 * You can instantiate another robot by passing the correct parameter to the constructor
 */
RobotController::RobotController(std::string arm_id) :
robot_controller_nh_("/ariac/"+arm_id),
robot_controller_options("manipulator",
        "/ariac/"+arm_id+"/robot_description",
        robot_controller_nh_),
robot_move_group_(robot_controller_options) {
    ROS_WARN(">>>>> RobotController");

    robot_move_group_.setPlanningTime(10);
    robot_move_group_.setNumPlanningAttempts(10);
    robot_move_group_.setPlannerId("RRTConnectkConfigDefault");
    robot_move_group_.setMaxVelocityScalingFactor(1.0);
    robot_move_group_.setMaxAccelerationScalingFactor(1.0);
    // robot_move_group_.setEndEffector("moveit_ee");
    robot_move_group_.allowReplanning(true);
    // ros::Publisher display_publisher = node.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    // moveit_msgs::DisplayTrajectory display_trajectory;

    

    //--These are joint positions used for the home position
    home_joint_pose_ = {0.0, 3.11, -1.60, 2.0, 4.30, -1.53, 0};
    // home_joint_pose_ = {0.0, -1.57, -3.14, 0.0, 0.0, 0.0, 0};
    bin_drop_pose_ = {2.5, 3.11, -1.60, 2.0, 3.47, -1.53, 0};
    belt_drop_pose_ = {2.5, 0, -1.60, 2.0, 3.47, -1.53, 0};
    conveyor = {1.13, 0, -0.70, 1.65, 3.74, -1.56, 0};



    //-- offset used for picking up parts
    //-- For the pulley_part, the offset is different since the pulley is thicker
    offset_ = 0.025;

    //--topic used to get the status of the gripper
    gripper_subscriber_ = gripper_nh_.subscribe(
            "/ariac/arm1/gripper/state", 10, &RobotController::GripperCallback, this);
   
    SendRobotHome();

    robot_tf_listener_.waitForTransform("arm1_linear_arm_actuator", "arm1_ee_link",
                                            ros::Time(0), ros::Duration(10));
    robot_tf_listener_.lookupTransform("/arm1_linear_arm_actuator", "/arm1_ee_link",
                                           ros::Time(0), robot_tf_transform_);


    fixed_orientation_.x = robot_tf_transform_.getRotation().x();
    fixed_orientation_.y = robot_tf_transform_.getRotation().y();
    fixed_orientation_.z = robot_tf_transform_.getRotation().z();
    fixed_orientation_.w = robot_tf_transform_.getRotation().w();
    ROS_INFO_STREAM(fixed_orientation_.x);
    ROS_INFO_STREAM(fixed_orientation_.y);
    ROS_INFO_STREAM(fixed_orientation_.z);
    ROS_INFO_STREAM(fixed_orientation_.w);

    // fixed_end_orientation_.x=-0.471938;
    // fixed_end_orientation_.y=0.517214;
    // fixed_end_orientation_.z=0.512411;
    // fixed_end_orientation_.w=0.497192;



    // ROS_INFO_STREAM(fixed_orientation_);




    tf::quaternionMsgToTF(fixed_orientation_,q);
    tf::Matrix3x3(q).getRPY(roll_def_,pitch_def_,yaw_def_);
        ROS_INFO_STREAM(roll_def_);
    ROS_INFO_STREAM(pitch_def_ );
    ROS_INFO_STREAM(yaw_def_);


    end_position_ = home_joint_pose_;
//   end_position_[0] = 2.5;
// //    end_position_[1] = 1.55;
// //    end_position_[2] = 1.2;


    robot_tf_listener_.waitForTransform("world", "arm1_ee_link", ros::Time(0),
                                            ros::Duration(10));
    robot_tf_listener_.lookupTransform("/world", "/arm1_ee_link", ros::Time(0),
                                           robot_tf_transform_);

    home_cart_pose_.position.x = robot_tf_transform_.getOrigin().x();
    home_cart_pose_.position.y = robot_tf_transform_.getOrigin().y();
    home_cart_pose_.position.z = robot_tf_transform_.getOrigin().z();
    home_cart_pose_.orientation.x = robot_tf_transform_.getRotation().x();
    home_cart_pose_.orientation.y = robot_tf_transform_.getRotation().y();
    home_cart_pose_.orientation.z = robot_tf_transform_.getRotation().z();
    home_cart_pose_.orientation.w = robot_tf_transform_.getRotation().w();

    agv_tf_listener_.waitForTransform("world", "kit_tray_1",
                                      ros::Time(0), ros::Duration(10));
    agv_tf_listener_.lookupTransform("/world", "/kit_tray_1",
                                     ros::Time(0), agv_tf_transform_);
    agv_position_.position.x = agv_tf_transform_.getOrigin().x();
    agv_position_.position.y = agv_tf_transform_.getOrigin().y();
    agv_position_.position.z = agv_tf_transform_.getOrigin().z() + 4 * offset_;

    gripper_client_ = robot_controller_nh_.serviceClient<osrf_gear::VacuumGripperControl>(
            "/ariac/arm1/gripper/control");
    // break_beam_subscriber_ = robot_controller_nh_.subscribe("/ariac/break_beam_1_change", 10, &RobotController::break_beam_callback_,this);

    counter_ = 0;
    drop_flag_ = false;


}

RobotController::~RobotController() {}

/**
 *
 * @return
 */
bool RobotController::Planner() {
    ROS_INFO_STREAM("Planning started...");
    if (robot_move_group_.plan(robot_planner_) ==
        moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        plan_success_ = true;
        ROS_INFO_STREAM("Planner succeeded!");
    } else {
        plan_success_ = false;
        ROS_WARN_STREAM("Planner failed!");
    }

    return plan_success_;
}


void RobotController::Execute() {
    ros::AsyncSpinner spinner(4);
    spinner.start();
    if (this->Planner()) {
        robot_move_group_.move();
        ros::Duration(0.5).sleep();
    }
}

void RobotController::GoToTarget(const geometry_msgs::Pose& pose, int f) {

    if (f==0){
    target_pose_.orientation=fixed_orientation_;
    }
    else if(f==1){
    target_pose_.orientation= conveyor_fixed_orientation_;

    }

    target_pose_.position = pose.position ;
    ros::AsyncSpinner spinner(4);
    robot_move_group_.setPoseTarget(target_pose_);
    spinner.start();
    if (this->Planner()) {
        robot_move_group_.move();
        ros::Duration(0.5).sleep();
    }
    ROS_INFO_STREAM("Point reached...");
}

void RobotController::GoToTarget(
        std::initializer_list<geometry_msgs::Pose> list, int f) {
    ros::AsyncSpinner spinner(4);
    spinner.start();

    std::vector<geometry_msgs::Pose> waypoints;
    if(f==0)
    {
    for (auto i : list) {
        i.orientation.x = fixed_orientation_.x;
        i.orientation.y = fixed_orientation_.y;
        i.orientation.z = fixed_orientation_.z;
        i.orientation.w = fixed_orientation_.w;
        waypoints.emplace_back(i);
    }}
    else if(f==1)
    {
          for (auto i : list) {
        i.orientation.x = conveyor_fixed_orientation_.x;
        i.orientation.y = conveyor_fixed_orientation_.y;
        i.orientation.z = conveyor_fixed_orientation_.z;
        i.orientation.w = conveyor_fixed_orientation_.w;
        waypoints.emplace_back(i);
    }  
    }

    moveit_msgs::RobotTrajectory traj;
    auto fraction =
            robot_move_group_.computeCartesianPath(waypoints, 0.01, 0.0, traj, true);

    ROS_WARN_STREAM("Fraction: " << fraction * 100);
    ros::Duration(0.5).sleep();

    robot_planner_.trajectory_ = traj;

    //if (fraction >= 0.3) {
        robot_move_group_.execute(robot_planner_);
        ros::Duration(0.5).sleep();
//    } else {
//        ROS_ERROR_STREAM("Safe Trajectory not found!");
//    }
}

void RobotController::SendRobotHome() {
    // ros::Duration(2.0).sleep();
    robot_move_group_.setJointValueTarget(home_joint_pose_);
    // this->execute();
    ros::AsyncSpinner spinner(4);
    spinner.start();
    if (this->Planner()) {
        robot_move_group_.move();
        ros::Duration(0.5).sleep();
    }
     ros::Duration(0.5).sleep();
}


void RobotController::SendRobotPosition(std::vector<double> pose) {
    // ros::Duration(2.0).sleep();
    robot_move_group_.setJointValueTarget(pose);
    // this->execute();
    ros::AsyncSpinner spinner(4);
    spinner.start();
    if (this->Planner()) {
        robot_move_group_.move();
        ros::Duration(1.5).sleep();
    }
     ros::Duration(0.5).sleep();
}

void RobotController::GripperToggle(const bool& state) {
    gripper_service_.request.enable = state;
    gripper_client_.call(gripper_service_);
    ros::Duration(1.0).sleep();
    // if (gripper_client_.call(gripper_service_)) {
    if (gripper_service_.response.success) {
        ROS_INFO_STREAM("Gripper activated!");
    } else {
        ROS_WARN_STREAM("Gripper activation failed!");
    }
}

bool RobotController::DropPart(geometry_msgs::Pose part_pose) {
  counter_++;

  pick = false;
  drop = true;

  ROS_WARN_STREAM("Dropping the part number: " << counter_);



  ROS_INFO_STREAM("Moving to end of conveyor...");
  robot_move_group_.setJointValueTarget(bin_drop_pose_);
  this->Execute();
  ros::Duration(1.0).sleep();
  // this->GripperStateCheck(part_pose);




  // if (drop == false) {
  //   ROS_INFO_STREAM("I am stuck here..." << object);
  //   ros::Duration(2.0).sleep();
  //   return drop;
  // }
  // ROS_INFO_STREAM("Dropping on AGV...");



  // agv_position_.position.x -= 0.1;
  // if (counter_ == 1) {
  //   agv_position_.position.y -= 0.1;
  // }
  // if (counter_ >= 2) {
  //   agv_position_.position.y += 0.1;
  //   // agv_position_.position.x +=0.1;
  // }


  part_pose.position.z += 0.1;
  auto temp_pose = part_pose;
  // auto temp_pose = agv_position_;
  temp_pose.position.z += 0.35;



  // temp_pose.position.y += 0.5;

  // this->setTarget(part_pose);
  // this->execute();
  // ros::Duration(1.0).sleep();


  this->GoToTarget({temp_pose, part_pose},1);
  // ros::Duration(0.5).sleep();
  ROS_INFO_STREAM("Dropping");
  this->GripperToggle(false);



  ROS_INFO_STREAM("Moving to end of conveyor...");
  robot_move_group_.setJointValueTarget(bin_drop_pose_);
  this->Execute();
  ros::Duration(1.0).sleep();

  // if(faulty-==true)
  // {

  // }






  // this->sendRobotHome();
  // temp_pose = home_cart_pose_;
  // temp_pose.position.z -= 0.05;


  // this->GoToTarget({temp_pose, home_cart_pose_},1);
  return drop;
}

// bool RobotController::DropPart(geometry_msgs::Pose part_pose) {
//     // counter_++;
//     // SendRobotPosition();
//     drop_flag_ = true;

//     ros::spinOnce();
//     ROS_INFO_STREAM("Placing phase activated...");

//     if (gripper_state_){//--while the part is still attached to the gripper
//         //--move the robot to the end of the rail
//          ROS_INFO_STREAM("Moving towards AGV1...");
         
//          part_pose.position.z += 0.1;

//          GoToTarget(part_pose,1);
//          // SendRobotHome();
//          // robot_move_group_.setJointValueTarget(end_position_);
//          this->Execute();
//          ros::Duration(0.5).sleep();
//          ROS_INFO_STREAM("Actuating the gripper...");
//          this->GripperToggle(false);




//        auto temp_pose = part_pose;
//        temp_pose.position.z += 0.5;
//        this->GoToTarget({temp_pose, part_pose});
//        ros::Duration(5).sleep();
//        ros::spinOnce();
//
//
//        ROS_INFO_STREAM("Actuating the gripper...");
//        this->GripperToggle(false);
//
//        ros::spinOnce();
//        if (!gripper_state_) {
//            ROS_INFO_STREAM("Going to home position...");
//            this->GoToTarget({temp_pose, home_cart_pose_});
//            ros::Duration(3.0).sleep();
//        }
// 




    

//     drop_flag_ = false;
//     return gripper_state_;
// }

void RobotController::GripperCallback(
        const osrf_gear::VacuumGripperState::ConstPtr& grip) {
    gripper_state_ = grip->attached;
}


bool RobotController::PickPart(geometry_msgs::Pose& part_pose) {
    // gripper_state = false;
    // pick = true;
    //ROS_INFO_STREAM("fixed_orientation_" << part_pose.orientation = fixed_orientation_);
    //ROS_WARN_STREAM("Picking the part...");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface group(robot_controller_options);

  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = robot_move_group_.getPlanningFrame();

  // The id of the object is used to identify it.
  collision_object.id = "box1";

  // Define a box to add to the world.
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.6;
  primitive.dimensions[1] = 0.6;
  primitive.dimensions[2] = 0.55;

  // Define a pose for the box (specified relative to frame_id)
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = -0.3;
  box_pose.position.y = -1.916;
  box_pose.position.z = 0.45;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);
  ROS_INFO_NAMED("tutorial", "Add an object into the world");
   planning_scene_interface.applyCollisionObjects(collision_objects);

    ROS_INFO_STREAM("Moving to part...");
    part_pose.position.z = part_pose.position.z + 0.033;
    // part_pose.position.x = part_pose.position.x - 0.02;
    // part_pose.position.y = part_pose.position.y - 0.02;
    auto temp_pose_1 = part_pose;
    temp_pose_1.position.z += 0.2;

    this->GoToTarget({temp_pose_1, part_pose},0);
    // GoToTarget(part_pose);

    ROS_INFO_STREAM("Actuating the gripper..." << part_pose.position.z);
    this->GripperToggle(true);
    ros::spinOnce();
    while (!gripper_state_) {
        part_pose.position.z -= 0.01;
        this->GoToTarget({temp_pose_1, part_pose},0);
        // GoToTarget(part_pose);
        ROS_INFO_STREAM("Actuating the gripper...");
        this->GripperToggle(true);
        ros::spinOnce();
    }

    ROS_INFO_STREAM("Going to waypoint...");
    // part_pose.position.z += 0.2;
    GoToTarget(temp_pose_1,0);
    // GoToTarget(part_pose);
    return gripper_state_;
}

bool RobotController::PickPartconveyor(std::string product) {

    ROS_INFO_STREAM("Moving to Conveyor");
    GripperToggle(true);
    SendRobotPosition(conveyor);
    robot_tf_listener_.waitForTransform("arm1_linear_arm_actuator", "arm1_ee_link",
                                            ros::Time(0), ros::Duration(10));
    robot_tf_listener_.lookupTransform("/arm1_linear_arm_actuator", "/arm1_ee_link",
                                           ros::Time(0), robot_tf_transform_);


    conveyor_fixed_orientation_.x = 0.00386643;
    conveyor_fixed_orientation_.y = 0.700713;
    conveyor_fixed_orientation_.z = 0.0037416; 
    conveyor_fixed_orientation_.w = 0.713423;
   conveyor_part.position.x=1.218116;
   conveyor_part.position.y=2.214948;
   conveyor_part.position.z=0.955569-0.03;
   // GoToTarget(conveyor_part,1);
 
int v=0;
std::string temp;
//  while(!beam.getBeam() && beam.getpart()!=product){
//     if(beam.getpart()!=product){
//       ROS_INFO_STREAM("PRODUCT"<<product);
//       while(v==0)
//       {
//         ros::spinOnce();
//         if(beam.getBeam())
//         {
//           v=1;
//         }
//       }


//  }
// ros::spinOnce();
// }

while(!beam.getBeam())
{
  ros::spinOnce();

}
    GoToTarget(conveyor_part,1);
    ros::spinOnce();
    conveyor_part.position.z=0.955569+0.1;
    if(gripper_state_){   
    GoToTarget(conveyor_part,1);
    }
    return gripper_state_;
}

// void RobotController::break_beam_callback_(const osrf_gear::Proximity::ConstPtr &msg) {
//     if (msg->object_detected) {
//         ROS_INFO("Break beam 2 triggered.");
// ROS_INFO_STREAM("BEAM"<<beam_);        
// break_beam_counter_ += 1;
//         beam_=true;
//      }
//      else{
//         beam_=false;
//      }
// }

