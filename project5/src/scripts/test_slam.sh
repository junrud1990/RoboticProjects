#!/usr/bin/env bash
xterm -e "source ../../devel/setup.bash; roslaunch turtlebot_gazebo turtlebot_world.launch" &
#xterm -e "cd ../../; source devel/setup.bash; roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/workspace/projects/project5/src/map/simple.world"
sleep 5

xterm -e "source ../../devel/setup.bash; roslaunch turtlebot_gazebo gmapping_demo.launch" &
sleep 5

xterm -e "source ../../devel/setup.bash; roslaunch turtlebot_rviz_launchers view_navigation.launch;bash" &
sleep 5

xterm -e "source ../../devel/setup.bash; roslaunch turtlebot_teleop keyboard_teleop.launch"
