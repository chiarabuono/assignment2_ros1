#! /usr/bin/env python

import rospy 
import actionlib.msg
import actionlib 
import sys

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

import assignment_2_2024.msg
import assignment_2_2024.msg import PlanningActionFeedback
from assignment2_ros1.msg import RobotInfo

Robot = {"prev_x": 0, "prev_y" : 0, "x": 0, "y": 0}
PERIOD = 10

latest_feedback = None
def feedback_callback(feedback):
    global latest_feedback
    latest_feedback = feedback

# Creates a goal to send to the action server.
def sending_goal(client, target_x, target_y):

    goal = assignment_2_2024.msg.PlanningGoal()   
    goal.target_pose = PoseStamped()
    goal.target_pose.pose.position.x = target_x
    goal.target_pose.pose.position.y = target_y

    client.send_goal(goal, feedback_cb = feedback_callback)
    rospy.loginfo("Goal set and sent")


def asking_questions(client):
    user_input = input("Press 'q' to cancel the goal, 'f' to receive feedback, 'v' to know the average velocity or 'e' to exit: ")

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
    while True:
        coordinate = input(string)
        if coordinate == "e": return "exit"
        try:
            coordinate = float(coordinate)
            break
        except ValueError:
            print("Invalid input. Please enter a valid number.")
    return coordinate

# The node also publishes the robot position and velocity as a custom message (x,y, vel_x, vel_z), by relying on the values published on the topic /odom
def publish_robot_info(msg):
    robot_info = RobotInfo()
    robot_info.x = msg.pose.pose.position.x
    robot_info.y = msg.pose.pose.position.y
    robot_info.vel_x = msg.twist.twist.linear.x
    robot_info.vel_z = msg.twist.twist.angular.z

    status_pub.publish(robot_info)

def reach_feedback(msg):
    if (msg.feedback.stat == "Target reached"):
        rospy.loginfo("Reached")


def publish_warning():
    global regions_
    
    if (regions_["fleft"] < 1): warning_pub.publish(True)
    elif (regions_["fright"] < 1): warning_pub.publish(True)
    elif (regions_["front"] < 1): warning_pub.publish(True)
    elif (regions_["left"] < 1): warning_pub.publish(True)
    elif (regions_["right"] < 1): warning_pub.publish(True)
    else: warning_pub.publish(False)

def clbk_laser(msg):
    global regions_
    regions_ = {
        'right':  min(min(msg.ranges[0:143]), 10),
        'fright': min(min(msg.ranges[144:287]), 10),
        'front':  min(min(msg.ranges[288:431]), 10),
        'fleft':  min(min(msg.ranges[432:575]), 10),
        'left':   min(min(msg.ranges[576:719]), 10),
    }

    publish_warning()




if __name__ == '__main__':
    try:
        # Initializes a rospy node
        rospy.init_node("action_client_node")

        status_pub = rospy.Publisher("/robot_status", RobotInfo, queue_size=10)
        rospy.Subscriber("/odom", Odometry, publish_robot_info)

        rospy.Subscriber("/reaching_goal/feedback", PlanningActionFeedback, reach_feedback)
        warning_pub = rospy.Publisher("/warning", bool, queue_size=10)
        sub_laser = rospy.Subscriber('/scan', LaserScan, clbk_laser)
        

        rate = rospy.Rate(PERIOD)

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

