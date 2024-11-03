#!/bin/bash
python3 odom_to_path.py &
rosbag play straight_path.bag
#rosbag play 1_rectangle.bag --start 15.44854 &
#python3 odom_pub.py &
wait
