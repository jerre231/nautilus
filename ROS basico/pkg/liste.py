#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

def calculate_and_publish_magnitude(data):
    modulo_linear = abs(data.linear.x) + abs(data.linear.y) + abs(data.linear.z)
    modulo_angular = abs(data.angular.x) + abs(data.angular.y) + abs(data.angular.z)
    magnitude = modulo_linear + modulo_angular
    rospy.loginfo(f"\n\nModulo linear: {modulo_linear}\nModulo angular: {modulo_angular}\nMagnitude: {magnitude}\n")
    magnitude_msg = Float32()
    modulol_msg = Float32()
    moduloa_msg = Float32()
    magnitude_msg.data = magnitude
    modulol_msg.data = modulo_linear
    moduloa_msg.data = modulo_angular
    pub.publish(magnitude_msg)
    pub.publish(modulol_msg)
    pub.publish(moduloa_msg)

def subscriber():
    rospy.init_node('velocity_magnitude_subscriber', anonymous=True)

    rospy.Subscriber('velocidades', Twist, calculate_and_publish_magnitude)

    rospy.spin()

if __name__ == '__main__':
    pub = rospy.Publisher('modulos_vel', Float32, queue_size=10)
    try:
        subscriber()
    except rospy.ROSInterruptException:
        pass

