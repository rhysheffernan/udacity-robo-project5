#!/bin/sh
pkill -f ros
pkill -f rviz
pkill -f gaz
terminator -e "source /opt/ros/kinetic/setup.bash; source ../../devel/setup.bash; roscore" & 
sleep 5
terminator -e "source /opt/ros/kinetic/setup.bash; source ../../devel/setup.bash; roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 5
#xterm  -e  " rosrun rviz rviz" &
terminator  -e  " source /opt/ros/kinetic/setup.bash; source ../../devel/setup.bash; roslaunch turtlebot_rviz_launchers view_navigation.launch;bash" &
sleep 5
terminator  -e  " source /opt/ros/kinetic/setup.bash; source ../../devel/setup.bash; roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$(rospack find turtlebot_gazebo)/maps/playground.yaml" &

