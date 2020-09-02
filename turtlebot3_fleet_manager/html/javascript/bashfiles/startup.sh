#!/bin/bash
source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash
export ROS_HOSTNAME=$1
export ROS_IP=$1
export ROS_MASTER_URI=http://$1:11311
roslaunch rosbridge_server rosbridge_websocket.launch &
rosrun web_video_server web_video_server _port:=8082


