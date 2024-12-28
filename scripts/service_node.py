#! /usr/bin/env python

import rospy 
import actionlib 
import sys

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can publish and subscribe over ROS.
        rospy.init_node("service_node")
        
        # A service node that, when called, returns the coordinates of the last target sent by the user;


    except rospy.ROSInterruptException:
        print("Action client interrupted", file=sys.stderr)