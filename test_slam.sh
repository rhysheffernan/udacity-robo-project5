#!/bin/sh
pkill -f ros
pkill -f rviz
pkill -f gaz
xterm  -e  " source devel/setup.bash; roscore" & 
sleep 5
xterm  -e  " roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 5
#xterm  -e  " rosrun rviz rviz" &
xterm  -e  " roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5
xterm  -e  " rosrun gmapping slam_gmapping" &
sleep 5
xterm  -e  " roslaunch turtlebot_teleop keyboard_teleop.launch" &
