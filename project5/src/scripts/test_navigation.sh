#!/usr/bin/env bash
xterm -e "source ../../devel/setup.bash; roslaunch turtlebot_gazebo turtlebot_world.launch;bash" &
#xterm -e "cd ../../; catkin_make; echo 0; source devel/setup.bash; roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/workspace/projects/project5/src/map/simple.world"
sleep 5

xterm -e "source ../../devel/setup.bash; roslaunch turtlebot_gazebo amcl_demo.launch;bash" &
sleep 5

xterm -e "source ../../devel/setup.bash; roslaunch turtlebot_rviz_launchers view_navigation.launch;bash"
