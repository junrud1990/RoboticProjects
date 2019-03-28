# Project 1
---
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/home/workspace/project1/build

# Project 2
---
roslaunch my_robot world.launch
roslaunch ball_chaser ball_chaser.launch

# Project 3
---
rostopic info/list/echo
roslaunch my_robot world.launch
roslaunch my_robot amcl.launch
rosrun teleop_twist_keyboard teleop_twist_keyboard.py

# Project 4
---
roslaunch my_robot world.launch
roslaunch my_robot mapping.launch
roslaunch my_robot teleop.launch
rtabmap-databaseViewer ~/.ros/rtabmap.db

# Project 5
---
sudo apt-get install ros-kinetic-turtlebot
sudo apt-get install ros-kinetic-gmapping
