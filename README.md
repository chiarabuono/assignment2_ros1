# Assignment 2 Research Track 1
This repository contains two ROS1 nodes that interact with the Gazebo environment:
1. Action Node: Allows the user to set a target position (x, y), cancel it, or receive feedback about the robot's pose.
2. Service Node: Returns the coordinates of the last target sent by the user when called.

# Requirements
Before using the package, make sure you have ROS (any compatible version) installed. Additionally, you must have a working catkin workspace to compile and run the package. If you do not already have a workspace, follow these steps to create a new one:
```
mkdir -p ~/ros_ws/src
gedit ~/.bashrc
```
and add this line ```source /root/ros_ws/devel/setup.bash``` to the bashrc file, where /root/ros_ws is the path of the folder. Then, close and reopen the terminal to make the changes effective.

Make sure you have Gazebo installed as well, as it is required to run the simulation.
# Package installation
1.	Go inside the src folder ```cd ~/ros_ws/src```
2.	Clone the repository into your workspace: ```git clone git@github.com:chiarabuono/assignment2_ros1.git```
3.	Clone the assignement2_2024 package into your workspace. This package contains the action server required: ```git clone git@github.com:CarmineD8/assignment_2_2024.git```
4.	Build the workspace in the main folder:
```
cd ~/ros_ws
catkin_make
```
# How to Run the package
Once you've installed the package, in the terminal, run the following command to launch both the action server node and the client node:
```
roslaunch assignment2_ros1 assignment2.launch 
```
This launch file will start:
- The action server node;
- The action client node;
- The launch file in the assignment2_2024 repository that will start the gazebo simulation and these nodes: wall_follower, go_to_point and bug_action_service.

# Node description
## Action node
This node allows the user to set a target's coordinates (x, y). If the values are invalid, the user will be prompted to reselect. Once valid coordinates are provided, the robot will move towards the target with a velocity established by the action server. The robot will stop when it reaches the target.
During the robot's motion, the user can interact with the following options:
-	Cancel the goal (digit q);
-	Receive feedback (digit f);
-	Or cancel the goal and exit the node with security (digit e).
## Service Node
This node allows the user to retrieve the last target coordinates set. It works regardless of whether the robot is currently reaching the target, has reached it, or if the goal was cancelled.
To access the last target set, use the service call:
```
rosservice call /last_target_service
```
