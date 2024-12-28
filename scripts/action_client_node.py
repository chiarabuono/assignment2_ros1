#! /usr/bin/env python

import rospy 
import actionlib 
import actionlib_tutorials.msg
from assignment2_ros1.msg import ReachTargetAction

def action_client():
    client = actionlib.SimpleActionClient('reachtarget', ReachTargetAction)
    client.wait_for_server()

    # Creates a goal to send to the action server.
    #goal = actionlib_tutorials.msg.FibonacciGoal(order=20)

    # Sends the goal to the action server.
    #client.send_goal(goal)

    # Waits for the server to finish performing the action.
    #client.wait_for_result()

    # Prints out the result of executing the action
    #return client.get_result()

def set_target(): 
    while True:
        try:
            x = float(input("Enter the target X coordinate: "))
            y = float(input("Enter the target Y coordinate: "))
            break
        except ValueError:
            print("Invalid input. Please enter a valid number.")
    return x, y


if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can publish and subscribe over ROS.
        rospy.init_node("action_client_node")
        rospy.sleep(2)

        # A node that implements an action client, allowing the user to set a target (x, y) 
        target_x, target_y = set_target()
        rospy.loginfo("Result: (%f, %f)", target_x, target_y)
        
        # or to cancel it. 

        # Try to use the feedback/status of the action server to know when the target has been reached. 
        result = action_client()

        # The node also publishes the robot position and velocity as a custom message (x,y, vel_x, vel_z), by relying on the values published on the topic /odom

    except rospy.ROSInterruptException:
        print("Action client interrupted", file=sys.stderr)





