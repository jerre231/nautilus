#!/usr/bin/env python3

import rospy
import tf2_ros
import math
from PlanetParams.msg import PlanetParams


def broadcast_planets_transforms():
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    tf_broadcaster = tf2_ros.TransformBroadcaster()

    planet_params_list = rospy.get_param("~planet_params")

    rate = rospy.Rate(10)  # Taxa de atualização de 10 Hz

    while not rospy.is_shutdown():
        for planet_params in planet_params_list:
            planet_name = planet_params.name
            orbit_radius = planet_params.orbit_radius

            # Cálculo das coordenadas x e y na órbita circular
            current_time = rospy.Time.now()
            angle = (2 * math.pi * current_time.to_sec()) % (2 * math.pi)
            x = orbit_radius * math.cos(angle)
            y = orbit_radius * math.sin(angle)

            # Definindo a transformação do planeta em relação à estrela (origem)
            planet_transform = tf2_ros.Transform()
            planet_transform.translation.x = x
            planet_transform.translation.y = y
            planet_transform.translation.z = 0.0
            planet_transform.rotation.x = 0.0
            planet_transform.rotation.y = 0.0
            planet_transform.rotation.z = 0.0
            planet_transform.rotation.w = 1.0

            # Publicando a transformação TF do planeta
            tf_broadcaster.sendTransform(planet_transform,
                                         rospy.Time.now(),
                                         "star",  # Frame da estrela (origem)
                                         planet_name,
                                         "sun")  # Tempo de vida da transformação

        rate.sleep()


if __name__ == "__main__":
    rospy.init_node("broadcaster")
    broadcast_planets_transforms()
