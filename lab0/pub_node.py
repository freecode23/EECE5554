#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

if __name__ ==  '__main__':
    # 1. Initialize node with a name.
    rospy.init_node("draw_circle_pub_node")

    # 2. Publish to a topic with the data type.
    # use rostopic list to get the exact name of the topic.
    # Use rostopic info '<topic_name>' to get the message type
    # Add dependency in package.xml
    # Queue size is the buffer subscriber will get messages will get message from the buffer.
    pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)
    
    # 3. Do something X times per second.
    rate = rospy.Rate(2)

    # 4. If the node has not received shutdown request:
    while not rospy.is_shutdown():
        # Create and publish the message.
        msg = Twist()
        msg.linear.x = 2.0
        msg.angular.z = 1.0
        pub.publish(msg)

        # Keep the loop running at 2 Hz.
        rate.sleep()

    # 5. To run the node that shows the GUI, execute:
    # rosrun turtlesim turtlesim_node.
    # Also run the current node:
    # rosrun my_robot_controller pub_node.py