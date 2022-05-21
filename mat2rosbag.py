from scipy.io import loadmat
import pandas as pd
import numpy as np
import rosbag
import rospy
# from qualisys.msg import Subject
from nav_msgs.msg import Odometry


file = './data/2022-05-11_MAir/output_data/trial9_M_mat2rosbag.mat'
origin_file = "./data/2022-05-11_MAir/rosbag/mair_trial9_M.bag"
out_file = "./data/trial9_M.bag"

add_origin_rosbag = 1
start_together = 1
start_time = 0


mat_file = loadmat(file)


with rosbag.Bag(out_file,'w') as bag:

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


    for i in range(int(mat_file["frames_center"])):
        husky_center_msg = Odometry()
        husky_center_msg.header.stamp.secs = int(mat_file["secs_center"][i])
        husky_center_msg.header.stamp.nsecs = int(mat_file["nsecs_center"][i])
        husky_center_msg.header.seq = i
        husky_center_msg.header.frame_id = "map"
        husky_center_msg.pose.pose.position.x = float(mat_file["husky_center_x"][i])
        husky_center_msg.pose.pose.position.y = float(mat_file["husky_center_y"][i])
        husky_center_msg.pose.pose.position.z = float(mat_file["husky_center_z"][i])
        husky_center_msg.pose.pose.orientation.x = float(mat_file["husky_center_qx"][i])
        husky_center_msg.pose.pose.orientation.y = float(mat_file["husky_center_qy"][i])
        husky_center_msg.pose.pose.orientation.z = float(mat_file["husky_center_qz"][i])
        husky_center_msg.pose.pose.orientation.w = float(mat_file["husky_center_qw"][i])

        timestamp = rospy.Time.from_sec(mat_file['timestamps_center'][i])
        if i == 0:
            start_time = timestamp
        bag.write("/qualisys_synced/husky_center_odom",husky_center_msg,timestamp)


    # Include the wrigin package:
    if (add_origin_rosbag):
        with rosbag.Bag(origin_file, 'r') as ib:
            print("Reading File: ", origin_file)
            for topic, msg, t in ib:
                if start_together and t >= start_time:
                    bag.write(topic, msg, t)
                elif not start_together:
                    bag.write(topic, msg, t)
print("Finished!")
