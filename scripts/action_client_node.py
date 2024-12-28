#! /usr/bin/env python

import actionlib.msg
import rospy 
import actionlib 
import sys

import assignment_2_2024.msg
from geometry_msgs.msg import PoseStamped

def send_goal(target_x, target_y):
    client = actionlib.SimpleActionClient('reaching_goal', assignment_2_2024.msg.PlanningAction)
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = assignment_2_2024.msg.PlanningGoal()   
    goal.target_pose = PoseStamped()
    goal.target_pose.pose.position.x = target_x
    goal.target_pose.pose.position.y = target_y
    rospy.loginfo("Goal set")

    client.send_goal(goal)
    rospy.loginfo("Goal sended")

    # Waits for the server to finish performing the action.
    client.wait_for_result()
    rospy.loginfo("Resulted received")

    # Prints out the result of executing the action
    return client.get_result()

def set_coordinate(string): 
    while True:
        try:
            coordinate = int(input(string))
            break
        except ValueError:
            print("Invalid input. Please enter a valid number.")
    return coordinate


if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can publish and subscribe over ROS.
        rospy.init_node("action_client_node")
        rospy.sleep(2)

        # A node that implements an action client, allowing the user to set a target (x, y) 
        target_x = set_coordinate("Enter the target X coordinate: ")
        target_y = set_coordinate("Enter the target Y coordinate: ")
        rospy.loginfo("Result: (%f, %f)", target_x, target_y)
        
        # or to cancel it. 

        # Try to use the feedback/status of the action server to know when the target has been reached. 
        result = send_goal(target_x, target_y)

        # The node also publishes the robot position and velocity as a custom message (x,y, vel_x, vel_z), by relying on the values published on the topic /odom

    except rospy.ROSInterruptException:
        print("Action client interrupted", file=sys.stderr)





