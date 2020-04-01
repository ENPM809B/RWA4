//
// Created by zeid on 2/27/20.
//

#pragma once


#include <list>
#include <map>
#include <string>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <osrf_gear/Proximity.h>
#include <tf/transform_listener.h>
#include <osrf_gear/LogicalCameraImage.h>

#include "ariac_part_manager.h"


class AriacSensorManager {
public:
    AriacSensorManager();
    ~AriacSensorManager();
    void LogicalCamera1Callback(const osrf_gear::LogicalCameraImage::ConstPtr&);
    void LogicalCamera2Callback(const osrf_gear::LogicalCameraImage::ConstPtr &);
    void LogicalCamera3Callback(const osrf_gear::LogicalCameraImage::ConstPtr &);
    void break_beam_callback_(const osrf_gear::Proximity::ConstPtr &);
    bool getBeam();
    void qualityControlSensor1Callback(const osrf_gear::LogicalCameraImage::ConstPtr &image_msg);
    
    // std::map<std::string, std::vector<std::string>> product_frame_list_;


    geometry_msgs::Pose GetPartPose(const std::string& src_frame,
                                    const std::string& target_frame);
    std::map<std::string, std::vector<std::string>> get_product_frame_list(){
        return product_frame_list_;
    }
    //void ScanParts(int cam_number);
    void BuildProductFrames(int camera_id);
    std::string getpart();

private:
    ros::NodeHandle sensor_nh_;
    ros::Subscriber camera_1_subscriber_;
    ros::Subscriber camera_2_subscriber_;
    ros::Subscriber camera_3_subscriber_;
    ros::Subscriber break_beam_subscriber_;
    ros::Subscriber quality_control_camera_subscriber_;


    tf::TransformListener camera_tf_listener_;
    tf::StampedTransform camera_tf_transform_;

    osrf_gear::LogicalCameraImage current_parts_1_;
    osrf_gear::LogicalCameraImage current_parts_2_;
    osrf_gear::LogicalCameraImage current_parts_3_;
    std::map<std::string, std::vector<geometry_msgs::Pose>> part_list_;
    std::vector<AriacPartManager> camera1_part_list,camera2_part_list,camera3_part_list;

    //std::map<std::string, std::list<std::string>> parts_list_;
    std::map<std::string, std::vector<std::string>> product_frame_list_;

    bool init1_,init2_,init3_, cam_1_, cam_2_,cam_3_,beam_;
    std::string logical1_,logical2_,logical3_;
    int camera1_frame_counter_, camera2_frame_counter_, camera3_frame_counter_,break_beam_counter_,prev1,prev2,prev3,f;
};

