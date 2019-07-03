#!/bin/sh
pkill -f ros
pkill -f rviz
pkill -f gaz
xterm  -e  " source devel/setup.bash; roscore" & 
sleep 5
xterm  -e  " roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 5
xterm  -e  " roslaunch turtlebot_navigation amcl_demo.launch map_file:=/home/robond/Documents/udacity-robo-project5/src/turtlebot_simulator/turtlebot_gazebo/maps/playground.yaml" &
#xterm  -e  " roslaunch turtlebot_navigation amcl_demo.launch map_file:=./src/turtlebot_simulator/turtlebot_gazebo/maps/playground.yaml" &
sleep 5
xterm  -e  " roslaunch turtlebot_rviz_launchers view_navigation.launch" &

