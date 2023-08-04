#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import random

def publisher():
    rospy.init_node('velocities_publisher', anonymous=True)
    rate = rospy.Rate(1)

    pub = rospy.Publisher('velocidades', Twist, queue_size=10)

    while not rospy.is_shutdown():
        twist_msg = Twist()
        twist_msg.linear.x = random.uniform(-1, 1)
        twist_msg.linear.y = random.uniform(-1, 1)
        twist_msg.linear.z = random.uniform(-1, 1)
        twist_msg.angular.x = random.uniform(-1, 1)
        twist_msg.angular.y = random.uniform(-1, 1)
        twist_msg.angular.z = random.uniform(-1, 1)

        pub.publish(twist_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass

