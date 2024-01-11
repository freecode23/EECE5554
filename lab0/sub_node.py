#! /usr/bin/env python3
import rospy
from turtlesim.msg import Pose

def pose_callback(msg):
    rospy.loginfo(msg)

if __name__ ==  '__main__':
    # 1. Initialize node with a name.
    rospy.init_node("draw_circle_sub_node")

    # 2. Subscribe to a pose topic.
    sub = rospy.Subscriber("/turtle1/pose", Pose, callback=pose_callback)
    
    # 3. Keep the node alive.
    # Can create another thread before spin if necessary.
    rospy.spin()