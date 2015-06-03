#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import division

import roslib
roslib.load_manifest('emily_nav')

import rospy
import numpy as np
np.set_printoptions(precision=1, suppress=True)

# from transformations import euler_from_quaternion

# Messages
from auv_msgs.msg import NavSts
from vehicle_interface.srv import BooleanService, BooleanServiceResponse
# xsens message


# Constants
R_EARTH = 6371000  # metres - average radius
A_EARTH = 6378137.0  # metres - semi-major axis - assumes elipsoidal model of Earth
B_EARTH = 6356752.3  # metres - semi-minor axis - assumes elipsoidal model of Earth

TOPIC_NAV = '/nav/nav_sts'
TOPIC_XSENS = '/xsens'
SRV_RESET_ORIGIN = '/nav/reset'
LOOP_RATE = 10  # Hz

class Navigation(object):
    def __init__(self, name, topic_nav):
        self.name = name

        self.point_ll = np.zeros(2)
        self.displacement_ne = np.zeros(2)

        self.origin_set = False
        #
        self.origin = np.zeros(2)  # [latitude, longitude] in radians
        # Distance to the centre of the Earth at given latitude assuming elipsoidal model of Earth.
        # If latitude is not known assume Earth is a sphere.
        self.geocentric_radius = R_EARTH

        # Subscribers
        # self.xsens_sub = rospy.Subscriber(TOPIC_XSENS, XSENS_MSG, self.handle_xsens)

        # Publishers
        self.nav_pub = rospy.Publisher(topic_nav, NavSts)

        # Services
        self.srv_reset = rospy.Service(SRV_RESET_ORIGIN, BooleanService, self.handle_reset)

    def loop(self):
        pass

    def handle_xsens(self, xsens_msg):
        try:
            nav_msg = NavSts()
            nav_msg.header.stamp = xsens_msg.header.stamp

            # global coords
            nav_msg.global_position.latitude = xsens_msg
            nav_msg.global_position.longitude = xsens_msg

            self.point_ll = np.array([nav_msg.global_position.latitude, nav_msg.global_position.longitude])

            # if origin is not set yet and we are at least 1 degree away from (0, 0)
            # WARN: This will cause issues when the system is used close to (0, 0) point (less than 150km)
            # TODO: find a better way to check this - potentially xkf field can inform about this
            if not self.origin_set and np.any(self.point_ll > 1):
                self.find_geo_origin(self.point_ll, self.displacement_ne)

            nav_msg.origin.latitude = self.origin[0]
            nav_msg.origin.longitude = self.origin[1]

            self.displacement_ne = geo2ne(self.point_ll, self.origin, self.geocentric_radius)

            # pose
            nav_msg.position.north = xsens_msg
            nav_msg.position.east = xsens_msg
            nav_msg.position.depth = xsens_msg
            nav_msg.orientation.roll = xsens_msg
            nav_msg.orientation.pitch = xsens_msg
            nav_msg.orientation.yaw = xsens_msg

            nav_msg.altitude = xsens_msg

            # pose change rate
            nav_msg.body_velocity.x = xsens_msg
            nav_msg.body_velocity.y = xsens_msg
            nav_msg.body_velocity.z = xsens_msg
            nav_msg.orientation_rate.roll = xsens_msg
            nav_msg.orientation_rate.pitch = xsens_msg
            nav_msg.orientation_rate.yaw = xsens_msg

            # add variances?

            self.nav_pub.publish(nav_msg)

        except Exception as e:
            rospy.logerr('%s', e)
            rospy.logerr('Bad xsens message format, skipping!')

    def handle_reset(self, srv):
        # set origin to where the vehicle is now
        if srv.request:
            self.origin = self.point_ll
            self.geocentric_radius = R_EARTH
        return BooleanServiceResponse(True)

    def find_geo_origin(self, point_ll, displacement_ne):
        """Finds the origin in geo-referenced frame. The origin corresponds to (0, 0) in ned reference frame
        (where xsens was started)

        :param point_ll: current position on Earth in spherical reference frame
        :param displacement_ne: current position in ne reference (displacement from where sensor was started)
        """
        radius = compute_geocentric_radius(point_ll[0])

        # NOTE: negative sign - that is because we want to look in the direction of origin
        self.origin = ne2geo(-displacement_ne, point_ll, radius)
        # find the radius corresponding to the new origin
        self.geocentric_radius = compute_geocentric_radius(self.origin[0])
        self.origin_set = True

def compute_geocentric_radius(latitude):
    """Finds distance to the centre of the Earth at the specified latitude. Refer to Wikipedia page:
        - http://en.wikipedia.org/wiki/Earth_radius

    :param latitude: latitude in degrees
    :return: local radius in metres
    """
    lat = np.deg2rad(latitude)
    radius = np.sqrt(((A_EARTH**2 * np.cos(lat))**2 + (B_EARTH**2 * np.sin(lat))**2) /
                     ((A_EARTH * np.cos(lat))**2 + (B_EARTH * np.sin(lat))**2))
    return radius

def geo2ne(point_ll, reference, geocentric_radius):
    """Given a point (latitude, longitude) and an origin the function returns a distance in n and e directions
    on a plane tangent to Earth at the origin point. n is along North. e is along East.
    This approximation is valid at latitudes close to the origin (how close exactly?). Zero altitude is assumed.

    Explanation of the applied method (WGS84) can be found in the MTi-g User Manual, page 31:
            - https://www.xsens.com/wp-content/uploads/2013/11/MTi-G_User_Manual_and_Technical_Documentation.pdf

    :param point_ll: a point on Earth (latitude, longitude) in degrees
    :param reference: a reference point on Earth (latitude, longitude) in degrees
    :param geocentric_radius: distance from the centre of the Earth to a point on the surface at a given latitude
            use compute_geocentric_radius to compute it. Can be approximated by specifying radius of spherical
            model of Earth
    :return: displacement_ne (in metres) - coordinates of point_ll in a cartesian reference frame where (0, 0) is at reference,
            n is along North, e is along East.
    """
    point_ll_rad = np.deg2rad(point_ll)
    reference_rad = np.deg2rad(reference)

    delta = point_ll_rad - reference_rad

    e = geocentric_radius * delta[1] * np.cos(reference_rad[0])
    n = geocentric_radius * delta[0]

    displacement_ne = np.array([n, e])

    return displacement_ne

def ne2geo(displacement_ne, reference, geocentric_radius):
    """Inverse of geo2ne(). Given a reference (latitude, longitude) and ne distance the function finds
    a point (latitude, longitude) displacement_ne away from the reference.

    :param displacement_ne: (n, e) distance along North and East.
    :param reference: a point on Earth (latitude, longitude) in degrees
    :param geocentric_radius: distance from the centre of the Earth to a point on the surface at a given latitude
            use compute_geocentric_radius to compute it. Can be approximated by specifying radius of spherical
            model of Earth
    :return: point_ll (in degrees) - point on Earth (latitude, longitude) displacement_ne away from the reference
    """
    reference_rad = np.deg2rad(reference)

    delta_lat = displacement_ne[0] / geocentric_radius
    delta_long = displacement_ne[1] / (np.cos(reference_rad[0]) * geocentric_radius)

    point_ll = np.array([reference[0] + np.rad2deg(delta_lat), reference[1] + np.rad2deg(delta_long)])

    return point_ll

if __name__ == '__main__':
    rospy.init_node('nav')
    name = rospy.get_name()

    topic_nav = rospy.get_param('~topic_nav', TOPIC_NAV)

    rospy.loginfo('nav topic: %s', topic_nav)

    nav = Navigation(name, topic_nav)
    loop_rate = rospy.Rate(LOOP_RATE)

    while not rospy.is_shutdown():
        try:
            nav.loop()
            loop_rate.sleep()
        except rospy.ROSInterruptException:
            rospy.loginfo('%s caught ros interrupt!', name)
        # except Exception as e:
        #     rospy.logfatal('%s', e)
        #     rospy.logfatal('Caught exception and dying!')
        #     sys.exit(-1)


