#!/usr/bin/env python3

import rospy
import tf2_ros
import math


def tf_listener_callback(tf_buffer, planet_name):
    try:
        trans = tf_buffer.lookup_transform("star", planet_name, rospy.Time())
        x = trans.transform.translation.x
        y = trans.transform.translation.y

        # Calcula a distância do planeta à estrela (origem)
        distance_to_star = math.sqrt(x**2 + y**2)

        rospy.loginfo(f"Planet {planet_name}: Distance to star = {distance_to_star:.2f} meters")
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        pass


if __name__ == "__main__":
    rospy.init_node("listener")

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    rate = rospy.Rate(1)  # Taxa de atualização de 1 Hz

    while not rospy.is_shutdown():
        # Subscreve as transformações TF para cada planeta
        planet_names = ["mercury", "venus", "earth"]  # Adicione mais planetas conforme necessário
        for planet_name in planet_names:
            tf_listener_callback(tf_buffer, planet_name)

        rate.sleep()
