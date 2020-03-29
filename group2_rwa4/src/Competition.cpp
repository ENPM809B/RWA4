#include<Competition.h>

/*
 * @brief
 *
 * @param
 *
 * @return
 */
Competition::Competition() = default;

/*
 * @brief
 *
 * @param
 *
 * @return
 */
void Competition::startCompetition(ros::NodeHandle & node) {
  ROS_INFO("Competition function.");
  // Create a Service client for the correct service, i.e. '/ariac/start_competition'.
  ros::ServiceClient start_client =
            node.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
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
  } else {
      ROS_INFO("Competition started!");
    }
}

/*
 * @brief
 *
 * @param
 *
 * @return
 */
void Competition::endCompetition(ros::NodeHandle & node) {
    ros::ServiceClient start_client =
            node.serviceClient<std_srvs::Trigger>("/ariac/end_competition");

    if (!start_client.exists()) {
        ROS_INFO("Waiting for the competition to be End...");
        start_client.waitForExistence();
        ROS_INFO("Competition  now Ended.");
    }
    ROS_INFO("Requesting competition End...");
    std_srvs::Trigger srv;
    start_client.call(srv);
    if (!srv.response.success) {
        ROS_ERROR_STREAM(
                "Failed to End the competition: " << srv.response.message);
    } else {
        ROS_INFO("Competition Ended!");
    }
}

// void Competition::logical_camera_callback(
//                 const osrf_gear::LogicalCameraImage::ConstPtr & image_msg)
//         {
//           geometry_msgs::TransformStamped tf_camera_wrt_world;
//         std::string product;
//           try{
//     	       tf_camera_wrt_world.transform.translation.x = image_msg->pose.position.x;
//     	       tf_camera_wrt_world.transform.translation.y = image_msg->pose.position.y;
//     	        tf_camera_wrt_world.transform.translation.z = image_msg->pose.position.z;
//     	         tf_camera_wrt_world.transform.rotation.w = image_msg->pose.orientation.w;
//     	          tf_camera_wrt_world.transform.rotation.x = image_msg->pose.orientation.x;
//     	           tf_camera_wrt_world.transform.rotation.y = image_msg->pose.orientation.y;
//     	            tf_camera_wrt_world.transform.rotation.z = image_msg->pose.orientation.z;
//
//         for(auto it=image_msg->models.begin(); it<image_msg->models.end(); ++it){
//         	geometry_msgs::Pose t_pose = it->pose;
//         	tf2::Quaternion q(t_pose.orientation.x,t_pose.orientation.y,t_pose.orientation.z,t_pose.orientation.w);
//             tf2::Matrix3x3 m(q);
//             double roll, pitch, yaw;
//         	m.getRPY(roll, pitch, yaw);
//           ROS_INFO_STREAM(it->type);
//
//
//         	// ROS_INFO("object in camera frame : [%f,%f,%f] [%f,%f,%f]", t_pose.position.x,
//         			// t_pose.position.y, t_pose.position.z, roll, pitch, yaw);
//         	tf2::doTransform(t_pose, t_pose, tf_camera_wrt_world);
//             q = tf2::Quaternion(t_pose.orientation.x,t_pose.orientation.y,t_pose.orientation.z,t_pose.orientation.w);
//             m = tf2::Matrix3x3(q);
//             m.getRPY(roll, pitch, yaw);
//             // ROS_INFO("object in world frame : [%f,%f,%f] [%f,%f,%f]", t_pose.position.x,
//                     			// t_pose.position.y, t_pose.position.z, roll, pitch, yaw);
//         }
//     }
//     catch (tf2::TransformException &ex) {
//     	ROS_WARN("%s",ex.what());
//         ros::Duration(1.0).sleep();
//     }
//         // for (auto msg: image_msg->models) {
//         //   ROS_INFO_STREAM(msg.type);
//         // }
//   }

/*
 * @brief
 *
 * @param
 *
 * @return
 */
Competition::~Competition() = default;
