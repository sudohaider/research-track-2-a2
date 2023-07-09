#!/bin/bash

gnome-terminal --tab --title="Gazebo simulation" -- bash -c "source /root/ros_ws/devel/setup.bash  sleep 2; roslaunch rt2_assignment1 sim_action.launch"
gnome-terminal --tab --title="go_to_point_action" -- bash -c " source /root/ros_ws/devel/setup.bash sleep 2; rosrun rt2_assignment1 go_to_point_action.py"

