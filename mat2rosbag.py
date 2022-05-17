from scipy.io import loadmat
import pandas as pd
import numpy as np
import rosbag
import rospy
# from qualisys.msg import Subject
from nav_msgs.msg import Odometry


file = 'data/trial_4_mat2rosbag.mat'
out_file = "data/trial_4.bag"
origin_file = "data/mair_trial4.bag"
add_origin_rosbag = 1

mat_file = loadmat(file)

with rosbag.Bag(out_file,'w') as bag:

    # Include the wrigin package:
    if (add_origin_rosbag):
        with rosbag.Bag(origin_file, 'r') as ib:
            print("Reading File: ", origin_file)
            for topic, msg, t in ib:
                bag.write(topic, msg, t)

    # Add new messages:
    print("Writing New Messages to File: ", out_file)
    for i in range(int(mat_file["frames_front"])): 
        husky_front_msg = Odometry()
        husky_front_msg.header.stamp.secs = int(mat_file["secs_front"][i])
        husky_front_msg.header.stamp.nsecs = int(mat_file["nsecs_front"][i])
        husky_front_msg.header.seq = i
        husky_front_msg.header.frame_id = "map"
        husky_front_msg.pose.pose.position.x = float(mat_file["husky_front_x"][i])
        husky_front_msg.pose.pose.position.y = float(mat_file["husky_front_y"][i])
        husky_front_msg.pose.pose.position.z = float(mat_file["husky_front_z"][i])
        husky_front_msg.pose.pose.orientation.x = float(mat_file["husky_front_qx"][i])
        husky_front_msg.pose.pose.orientation.y = float(mat_file["husky_front_qy"][i])
        husky_front_msg.pose.pose.orientation.z = float(mat_file["husky_front_qz"][i])
        husky_front_msg.pose.pose.orientation.w = float(mat_file["husky_front_qw"][i])

        timestamp = rospy.Time.from_sec(mat_file['timestamps_front'][i])
        bag.write("/qualisys_synced/husky_front_odom",husky_front_msg,timestamp)


    for i in range(int(mat_file["frames_wifi"])):
        husky_wifi_msg = Odometry()
        husky_wifi_msg.header.stamp.secs = int(mat_file["secs_wifi"][i])
        husky_wifi_msg.header.stamp.nsecs = int(mat_file["nsecs_wifi"][i])
        husky_wifi_msg.header.seq = i
        husky_wifi_msg.header.frame_id = "map"
        husky_wifi_msg.pose.pose.position.x = float(mat_file["husky_wifi_x"][i])
        husky_wifi_msg.pose.pose.position.y = float(mat_file["husky_wifi_y"][i])
        husky_wifi_msg.pose.pose.position.z = float(mat_file["husky_wifi_z"][i])
        husky_wifi_msg.pose.pose.orientation.x = float(mat_file["husky_wifi_qx"][i])
        husky_wifi_msg.pose.pose.orientation.y = float(mat_file["husky_wifi_qy"][i])
        husky_wifi_msg.pose.pose.orientation.z = float(mat_file["husky_wifi_qz"][i])
        husky_wifi_msg.pose.pose.orientation.w = float(mat_file["husky_wifi_qw"][i])

        timestamp = rospy.Time.from_sec(mat_file['timestamps_wifi'][i])
        bag.write("/qualisys_synced/husky_wifi_odom",husky_wifi_msg,timestamp)

print("Finished!")
