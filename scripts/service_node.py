#! /usr/bin/env python
# A service node that, when called, returns the coordinates of the last target sent by the user;
import rospy 
import sys

from assignment2_ros1.srv import LastTarget, LastTargetResponse

def retrieve_last_target(req):
    last_x = float(rospy.get_param("/des_pos_x"))
    last_y = float(rospy.get_param("/des_pos_y"))
    
    return LastTargetResponse(last_x, last_y)


if __name__ == '__main__':
    try:
        # Initializes a rospy node
        rospy.init_node("service_node")
        service = rospy.Service("/last_target_service", LastTarget, retrieve_last_target)
        
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            rate.sleep()

    except rospy.ROSInterruptException:
        print("Service node interrupted", file=sys.stderr)