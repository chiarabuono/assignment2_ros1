#! /usr/bin/env python
# A service node that, when called, returns the coordinates of the last target sent by the user;
import rospy 
import sys

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

from assignment2_ros1.srv import LastTarget, LastTargetResponse
from assignment2_ros1.srv import LastDistance, LastDistanceResponse

target_x, target_y = 0.0, 0.0
position_x, position_y = 0.0, 0.0

def retrieve_last_target(req):
    last_x = float(rospy.get_param("/des_pos_x"))
    last_y = float(rospy.get_param("/des_pos_y"))

    global target_x, target_y
    try:
        target_x = last_x
        target_y = last_y
    except:
        target_x = None
        target_y = None
    
    return LastTargetResponse(last_x, last_y)

def retrieve_last_position(msg):
    global position_x, position_y
    position_x = msg.pose.pose.position.x
    position_y = msg.pose.pose.position.y

def distance_last_target():
    rospy.Subscriber("/odom", Odometry, retrieve_last_position)

    global target_x, target_y, position_x, position_y
    if target_x is not None and target_y is not None:
        distance_x = abs(target_x - position_x)
        distance_y = abs(target_y - position_y)
    else:
        distance_x = 0
        distance_y = 0
    return LastDistanceResponse(distance_x, distance_y)

    
if __name__ == '__main__':
    try:
        # Initializes a rospy node
        rospy.init_node("service_node")
        service = rospy.Service("/last_target_service", LastTarget, retrieve_last_target)
        distanceFromLastTarget_srv = rospy.Service("/distance_last_target", LastDistance, distance_last_target)
        
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            rate.sleep()

    except rospy.ROSInterruptException:
        print("Service node interrupted", file=sys.stderr)