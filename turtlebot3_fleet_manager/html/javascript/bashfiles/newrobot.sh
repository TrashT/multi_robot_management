#!/bin/bash

source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://$1:11311
export ROS_HOSTNAME=$1
export ROS_IP=$1
export TURTLEBOT3_MODEL=$2
ROS_NAMESPACE=$3 roslaunch turtlebot3_fleet_manager turtlebot3_remote_custom.launch multi_robot_name:=$3 &
ROS_NAMESPACE=$3 rosrun map_server map_server /var/www/html/map/officeMap.yaml &
ROS_NAMESPACE=$3 roslaunch turtlebot3_fleet_manager amcl_custom.launch &
ROS_NAMESPACE=$3 roslaunch turtlebot3_fleet_manager move_base_custom.launch cmd_vel_topic:=cmd_vel &