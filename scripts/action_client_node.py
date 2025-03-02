#! /usr/bin/env python
"""
This node implements an action client that allows the user to set a target (x, y) or to cancel it. It interact with the user allowing to
cancel the goal, receive feedback or close the node. It also publishes the robot position and velocity as a custom message (x,y, vel_x, vel_z), 
by relying on the values published on the topic /odom.

.. module:: action_client_node
   :platform: Unix
   :synopsis: Action client node for setting and canceling a target (x, y).

.. moduleauthor:: Chiara Buono <s7687956@studenti.unige.it>
"""


import rospy 
import actionlib.msg
import actionlib 
import sys

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

import assignment_2_2024.msg
from assignment2_ros1.msg import RobotInfo


latest_feedback = None
def feedback_callback(feedback):
    """
    Function to handle feedback from the action server.

    :param feedback: The feedback message received from the action server.
    :type feedback: PlanningFeedback
    """
    global latest_feedback
    latest_feedback = feedback

def sending_goal(client, target_x, target_y):
    """
    Function to send a goal to the action server.

    :param client: The action client.
    :type client: ActionClient
    :param target_x: The x coordinate of the target.
    :type target_x: float
    :param target_y: The y coordinate of the target.
    :type target_y: float
    """

    goal = assignment_2_2024.msg.PlanningGoal()   
    goal.target_pose = PoseStamped()
    goal.target_pose.pose.position.x = target_x
    goal.target_pose.pose.position.y = target_y

    client.send_goal(goal, feedback_cb = feedback_callback)
    rospy.loginfo("Goal set and sent")


def asking_questions(client):
    """
    Function to handle the interaction with the user. It will allow the user to cancel the goal, receive feedback or close the node safely.

    :param client: The action client.
    :type client: ActionClient

    :returns: A string based on the choise of the user
    :rtype: string 
    """
    user_input = input("Press 'q' to cancel the goal, 'f' to receive feedback or 'e' to exit: ")

    # or to cancel it. 
    if user_input.lower() == "q":
        # and goal not reached
        if client.get_state() not in [actionlib.GoalStatus.SUCCEEDED, actionlib.GoalStatus.ABORTED, actionlib.GoalStatus.PREEMPTED]:
            rospy.loginfo("Cancelling goal")
            client.cancel_goal()
        else:
            rospy.loginfo("Goal already reached")
        return "targetcancelled"
        

    # Try to use the feedback/status of the action server to know when the target has been reached. 
    elif user_input.lower() == "f":
        rospy.loginfo("Getting feedback...")
        if latest_feedback is None:
            rospy.loginfo("Feedback still not received")
        else:
            rospy.loginfo("Feedback received: %s", latest_feedback)
    
    elif user_input.lower() == "e":
        if client.get_state() not in [actionlib.GoalStatus.SUCCEEDED, actionlib.GoalStatus.ABORTED, actionlib.GoalStatus.PREEMPTED]:
            rospy.loginfo("Cancelling goal and exiting")
            client.cancel_goal()
        else: rospy.loginfo("Exiting")
        return "exit"
    
    else:
        rospy.loginfo("Input not valid")

def set_coordinate(string): 
    """
    Function to convert the user input from string to float. It also handle the 'exit' case

    :param string: The user input
    :type string: string

    :returns: coordinate
    :rtype: float
    """
    while True:
        coordinate = input(string)
        if coordinate == "e": return "exit"
        try:
            coordinate = float(coordinate)
            break
        except ValueError:
            print("Invalid input. Please enter a valid number.")
    return coordinate

def publish_robot_info(msg):
    """
    Function that publishes the robot position and velocity as a custom message (x, y, vel_x, vel_z) by relying on the values published on the topic /odom

    :param msg: The message to be published
    :type msg: geometry_msgs.msg.TwistStamped

    """
    robot_info = RobotInfo()
    robot_info.x = msg.pose.pose.position.x
    robot_info.y = msg.pose.pose.position.y
    robot_info.vel_x = msg.twist.twist.linear.x
    robot_info.vel_z = msg.twist.twist.angular.z

    status_pub.publish(robot_info)

if __name__ == '__main__':
    try:
        # Initializes a rospy node
        rospy.init_node("action_client_node")

        status_pub = rospy.Publisher("/robot_status", RobotInfo, queue_size=10)
        rospy.Subscriber("/odom", Odometry, publish_robot_info)

        rate = rospy.Rate(10)

        while not rospy.is_shutdown():

            # A node that implements an action client, allowing the user to set a target (x, y) 
            print("Enter 'e' at any moment to exit")
            target_x = set_coordinate("Enter the target X coordinate: ")
            if target_x == "exit": break
            
            target_y = set_coordinate("Enter the target Y coordinate: ")
            if target_y == "exit": break

            
            client = actionlib.SimpleActionClient('reaching_goal', assignment_2_2024.msg.PlanningAction)
            client.wait_for_server()
            sending_goal(client, target_x, target_y)

            while client.get_state() not in [actionlib.GoalStatus.SUCCEEDED, actionlib.GoalStatus.ABORTED, actionlib.GoalStatus.PREEMPTED]:
                answer = asking_questions(client)
                rate.sleep()
                if answer == "exit" or answer == "targetcancelled":
                    break
            if answer == "exit":
                break
            rate.sleep()              # To allow having the question as last in the terminal

        rospy.loginfo("Exit success")

    except rospy.ROSInterruptException:
        print("Action client interrupted", file=sys.stderr)

