#! /usr/bin/env python3
import rospy
from std_msgs.msg import String

def talker():

    # 1. Initialize node with a name.
    rospy.init_node("talker")

    # 2. Publish to a topic with the data type.
    # use rostopic list to get the exact name of the topic.
    # Use rostopic info '<topic_name>' to get the message type
    # Add dependency in package.xml
    # Queue size is the buffer from where subscirber will the message.
    pub = rospy.Publisher("chatter", String, queue_size=10)
    
    # 3. Do something X times per second.
    rate = rospy.Rate(10)

    # 4. If the node has not received shutdown request:
    i = 0
    while not rospy.is_shutdown():
        # Create and publish the message.
        msg = "anything but hello for the " + str(i) + "th time "
        rospy.loginfo(msg)
        pub.publish(msg)
        i += 1

        # Keep the loop running at 10 Hz.
        rate.sleep()

if __name__ ==  '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass