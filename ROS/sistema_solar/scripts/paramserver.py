#!/usr/bin/env python3

import rospy
from rospy import get_param
from PlanetParams.msg import PlanetParams


def load_planet_params():
    planets = rospy.get_param("/planets")
    planet_params_list = []

    for planet in planets:
        name = planet["name"]
        orbit_radius = planet["orbit_radius"]
        planet_params_list.append(PlanetParams(name=name, orbit_radius=orbit_radius))

    return planet_params_list


if __name__ == "__main__":
    rospy.init_node("paramserver")
    planet_params_list = load_planet_params()
    rospy.set_param("~planet_params", planet_params_list)
    rospy.spin()
