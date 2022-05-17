#pragma once
#include "ros/ros.h"
#include <string>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <limits>
#include <vector>
#include <boost/bind.hpp>
// #include <mutex>
// #include <thread>
// #include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include "nav_msgs/Odometry.h"
#include <nav_msgs/Path.h>
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

#include <random>
// #include "qualisys/Subject.h"

std::string file_name_front = "data/trial_4_gt_path_front.txt";
std::string file_name_wifi = "data/trial_4_gt_path_wifi.txt";

int offset_sec = 0;
int offset_nsec = 0;

class QualisysListener {
public:
  QualisysListener(ros::NodeHandle& nh): nh_(nh)
  {
    
    husky_front_sub = nh.subscribe("/qualisys_synced/husky_front_odom", 1000, &QualisysListener::FrontOdomCallback, this);
    husky_wifi_sub = nh.subscribe("/qualisys_synced/husky_wifi_odom", 1000, &QualisysListener::WifiOdomCallback, this);

    // odom_front_pub = nh_.advertise<nav_msgs::Odometry>("/qualisys_synced/husky_front", 1000);
    // odom_wifi_pub = nh_.advertise<nav_msgs::Odometry>("/qualisys_synced/husky_wifi", 1000);
    path_front_pub = nh_.advertise<nav_msgs::Path>("/qualisys_synced/gt_path_front", 1000);
    path_wifi_pub = nh_.advertise<nav_msgs::Path>("/qualisys_synced/gt_path_wifi", 1000);
    
    std::ofstream refresh_outfile_front(file_name_front, std::ofstream::out);
    std::ofstream refresh_outfile_wifi(file_name_wifi, std::ofstream::out);

  }

  ~QualisysListener() {}


  void savePoseCallback(const geometry_msgs::Pose& pose, const long double timestamp, std::string& file_name) {
    if (file_name.size() > 0) {
        // ROS_INFO_STREAM("write new pose\n");

        // tum style
        std::ofstream tum_outfile(file_name, std::ofstream::out | std::ofstream::app );
        // std::cout.precision(dbl::max_digits10);
        // std::cout << timestamp << std::endl;
        tum_outfile.precision(20);
        tum_outfile << timestamp << " "<< pose.position.x <<" "<< pose.position.y << " "<< pose.position.z << " "<< pose.orientation.x\
        <<" "<< pose.orientation.y <<" "<< pose.orientation.z <<" "<< pose.orientation.w <<std::endl<<std::flush;
        
        tum_outfile.close();
    }
  }

  void FrontOdomCallback(const nav_msgs::Odometry& msg)
  {
    geometry_msgs::PoseStamped pose;
    pose.header = msg.header;
    pose.header.stamp.sec += offset_sec;
    pose.header.stamp.nsec += offset_nsec;
    pose.pose.position = msg.pose.pose.position;
    pose.pose.orientation = msg.pose.pose.orientation;

    if (poses_front.size() == 0) {
        initial_pose_front = pose.pose;
    } else {
      pose.pose.position.x -= initial_pose_front.position.x;
      pose.pose.position.y -= initial_pose_front.position.y;
      pose.pose.position.z -= initial_pose_front.position.z;

    }

    poses_front.push_back(pose);

    if (poses_front.size() % 10 == 0) {
        nav_msgs::Path path_msg;
        path_msg.header = msg.header;
        path_msg.header.frame_id = "/odom";
        std::cout << "Positions: " << poses_front.back().pose.position.x << ", " << poses_front.back().pose.position.x << ", " << poses_front.back().pose.position.z << std::endl;
        path_msg.poses = poses_front;
        
        long double timestamp;
        timestamp = msg.header.stamp.sec + (double)msg.header.stamp.nsec / (double)1000000000;
        savePoseCallback(pose.pose, timestamp, file_name_front);

        path_front_pub.publish(path_msg);
    }
  }

  void WifiOdomCallback(const nav_msgs::Odometry& msg)
  {
    geometry_msgs::PoseStamped pose;
    pose.header = msg.header;
    pose.header.stamp.sec += offset_sec;
    pose.header.stamp.nsec += offset_nsec;
    pose.pose.position = msg.pose.pose.position;
    pose.pose.orientation = msg.pose.pose.orientation;

    if (poses_wifi.size() == 0) {
        initial_pose_wifi = pose.pose;
    } else {
      pose.pose.position.x -= initial_pose_wifi.position.x;
      pose.pose.position.y -= initial_pose_wifi.position.y;
      pose.pose.position.z -= initial_pose_wifi.position.z;
    }

    poses_wifi.push_back(pose);

    if (poses_wifi.size() % 10 == 0) {
        nav_msgs::Path path_msg;
        path_msg.header = msg.header;
        path_msg.header.frame_id = "/odom";
        std::cout << "Positions: " << poses_wifi.back().pose.position.x << ", " << poses_wifi.back().pose.position.x << ", " << poses_wifi.back().pose.position.z << std::endl;
        path_msg.poses = poses_wifi;
        
        long double timestamp;
        timestamp = msg.header.stamp.sec + (double)msg.header.stamp.nsec / (double)1000000000;
        savePoseCallback(pose.pose, timestamp, file_name_wifi);

        path_wifi_pub.publish(path_msg);
    }
  }


private:
    ros::NodeHandle& nh_;
    std::vector<geometry_msgs::PoseStamped> poses_front;
    std::vector<geometry_msgs::PoseStamped> poses_wifi;

    ros::Subscriber husky_front_sub;
    ros::Subscriber husky_wifi_sub;

    ros::Publisher path_front_pub;
    ros::Publisher path_wifi_pub;

    geometry_msgs::Pose initial_pose_front;
    geometry_msgs::Pose initial_pose_wifi;

};

