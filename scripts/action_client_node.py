#! /usr/bin/env python

import actionlib.msg
import rospy 
import actionlib 
import sys

import assignment_2_2024.msg
from geometry_msgs.msg import PoseStamped

latest_feedback = None
def feedback_callback(feedback):
    global latest_feedback
    latest_feedback = feedback

# Creates a goal to send to the action server.
def send_goal(client, target_x, target_y):

    goal = assignment_2_2024.msg.PlanningGoal()   
    goal.target_pose = PoseStamped()
    goal.target_pose.pose.position.x = target_x
    goal.target_pose.pose.position.y = target_y
    rospy.loginfo("Goal set")

    client.send_goal(goal, feedback_cb = feedback_callback)
    rospy.loginfo("Goal sent")


def asking_questions(client):
    user_input = input("Press 'q' to cancel the goal, 'f' to receive feedback or 'e' to exit: ")
    
    # or to cancel it. 
    if user_input.lower() == "q":
        rospy.loginfo("Cancelling goal")
        client.cancel_goal()

    # Try to use the feedback/status of the action server to know when the target has been reached. 
    elif user_input.lower() == "f":
        rospy.loginfo("Getting feedback...")
        if latest_feedback is None:
            rospy.loginfo("Feedback still not received")
        else:
            rospy.loginfo("Feedback received: %s", latest_feedback)
    
    elif user_input.lower() == "e":
        rospy.loginfo("Cancelling goal and exting")
        client.cancel_goal()
        return "exit"
    
    else:
        rospy.loginfo("Input not valid")

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

        while not rospy.is_shutdown():
            # A node that implements an action client, allowing the user to set a target (x, y) 
            target_x = set_coordinate("Enter the target X coordinate: ")
            target_y = set_coordinate("Enter the target Y coordinate: ")
            rospy.loginfo("Target chosen: (%f, %f)", target_x, target_y)

            
            client = actionlib.SimpleActionClient('reaching_goal', assignment_2_2024.msg.PlanningAction)
            client.wait_for_server()
            send_goal(client, target_x, target_y)

            while client.get_state() != actionlib.GoalStatus.SUCCEEDED and client.get_state() != actionlib.GoalStatus.ABORTED and client.get_state() != actionlib.GoalStatus.PREEMPTED:
                answer = asking_questions(client)
                rospy.sleep(1)
            if answer == "exit":
                break

            rospy.sleep(0.5)    
 

            # The node also publishes the robot position and velocity as a custom message (x,y, vel_x, vel_z), by relying on the values published on the topic /odom
        rospy.loginfo("Exit success")

    except rospy.ROSInterruptException:
        print("Action client interrupted", file=sys.stderr)





