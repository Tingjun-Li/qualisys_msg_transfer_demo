# Qualisys mat to rosbag

## .mat to .bag
1. clone the repository into your catkin workspace
2. edit mat2rosbag.py according to your FILE.mat and FILE.bag setup
3. run mat2rosbag.py to write the rosbag

## qualisys data to path:
1. clone the repository intor your catkin workspace.
2. edit qualisys_pose.hpp accordingly. This will generate a path by using the qualisys data you wrote to the rosbag.
3. run `catkin_make` under `catkin/` folder, then `source ./devel/setup.bash`
4. run `roscore` in a new terminal
5. In the original terminal, run `rosrun qualisys_pose qualisys_listenser_node`
