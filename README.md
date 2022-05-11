# Qualisys mat to rosbag

## .mat to .bag
1. clone the repository into your catkin workspace
2. edit mat2rosbag.py according to your FILE.mat setup
3. run mat2rosbag.py to write the rosbag

## qualisys msg to nav_msgs/Odometry:
1. clone the repository intor your catkin workspace, the one you have generated qualisys msg type
2. edit qualisys_pose.hpp accordingly
3. run `catkin_make` under `catkin/` folder, then `source ./devel/setup.bash`
4. run `roscore` in a new terminal
5. In the original terminal, run `rosrun qualisys_pose qualisys_listenser_node`
