#!/bin/bash

# Launch the nodes
xterm  -e "source devel/setup.bash; export TURTLEBOT_GAZEBO_WORLD_FILE="$(pwd)/src/world/office.world"; roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 7
xterm  -e "source devel/setup.bash; export TURTLEBOT_GAZEBO_MAP_FILE="$(pwd)/src/world/office.yaml"; roslaunch turtlebot_gazebo amcl_demo.launch " &
sleep 5
xterm  -e "source devel/setup.bash; roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5
xterm  -e "source devel/setup.bash; rosrun pick_objects pick_objects"


