### pick_objects

Target planning task for the robot

* It targets the drop location
* Waits after reaching 5s to simulate a pick-up
* Moves to the second location

### add_markers

Adds markers to visualize the target location. This node uses a timer and displays the drop locations.

* First location will be shown for 5s
* Drop is hidden for 5s
* Second location will be shown next until reached

### SLAM

Simultaneous localization and mapping (SLAM) is the computational problem of constructing or updating a map of an unknown environment while simultaneously keeping track of an robots's location within it.
Popular approximate solution methods include the particle filter, extended Kalman filter, covariance intersection, and GraphSLAM. GMapping solves the Simultaneous Localization and Mapping (SLAM) problem by employing a particle filter. 

* https://github.com/ros-perception/slam_gmapping.git 

### Turtlebot:

TurtleBot is a low-cost, open source personal robot kit. 
The bot has rotary encoders, IMU and a camera and can be simulated thanks to gezebo plugins.
The turtlebot stack provides all the basic drivers for running and using a TurtleBot with ROS

* https://github.com/turtlebot/turtlebot.git
* https://github.com/turtlebot/turtlebot_interactions.git
* https://github.com/turtlebot/turtlebot_simulator.git

This is the evolution of the turtlebot_viz stack supporting user side interactions with the turtlebot.
Launchers for Gazebo simulation of the TurtleBot
