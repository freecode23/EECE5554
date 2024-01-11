#! /usr/bin/env python3
import rospy
from std_msgs.msg import String

def chatter_callback(msg):
    recv_data = msg.data
    recv_data += " received"
    rospy.loginfo(recv_data)

if __name__ ==  '__main__':
    # 1. Initialize node with a name.
    rospy.init_node("listener")

    # 2. Subscribe to a pose topic.
    sub = rospy.Subscriber("chatter", String, callback=chatter_callback)
    
    # 3. Keep the node alive.
    # Can create another thread before spin if necessary.
    rospy.spin()