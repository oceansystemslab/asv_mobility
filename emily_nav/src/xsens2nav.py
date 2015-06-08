#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import division

import roslib
roslib.load_manifest('emily_nav')

import rospy
import numpy as np
np.set_printoptions(precision=1, suppress=True)

import frame_maths as frame

# Messages
from auv_msgs.msg import NavSts
from xsens.msg import Xsens
from vehicle_interface.srv import BooleanService, BooleanServiceResponse
# xsens message

# Constants
R_EARTH = 6371000  # metres - average radius of Earth

TOPIC_NAV = '/nav/nav_sts/new'
TOPIC_XSENS = '/imu/xsens'
SRV_RESET_ORIGIN = '/nav/reset'
LOOP_RATE = 10  # Hz

SENSOR_ANGLE_OFFSETS = np.array([np.pi, 0, np.pi])

class Navigation(object):
    def __init__(self, name, topic_nav):
        self.name = name

        self.point_ll = np.zeros(2)
        self.displacement_ne = np.zeros(2)

        self.origin_set = False
        self.origin = np.zeros(2)  # [latitude, longitude] in radians

        # Distance to the centre of the Earth at given latitude assuming elipsoidal model of Earth.
        # If latitude is not known assume Earth is a sphere.
        self.geocentric_radius = R_EARTH

        # Subscribers
        self.xsens_sub = rospy.Subscriber(TOPIC_XSENS, Xsens, self.handle_xsens)

        # Publishers
        self.nav_pub = rospy.Publisher(topic_nav, NavSts)

        # Services
        self.srv_reset = rospy.Service(SRV_RESET_ORIGIN, BooleanService, self.handle_reset)

    def loop(self):
        # something to do here?
        pass

    def handle_xsens(self, xsens_msg):
        try:
            nav_msg = NavSts()
            nav_msg.header.stamp = xsens_msg.header.stamp

            # global coords
            nav_msg.global_position.latitude = xsens_msg.position.latitude
            nav_msg.global_position.longitude = xsens_msg.position.longitude

            self.point_ll = np.array([nav_msg.global_position.latitude, nav_msg.global_position.longitude])

            # if origin is not set yet and we are at least 1 degree away from (0, 0)
            # WARN: This will cause issues when the system is used close to (0, 0) point (less than 150km)
            # TODO: find a better way to check this - potentially xkf_valid field can inform about this
            # if not self.origin_set and xsens_msg.xkf_valid: - not reliable
            if not self.origin_set and np.any(self.point_ll > 1):
                self.find_geo_origin(self.point_ll, self.displacement_ne)

            nav_msg.origin.latitude = self.origin[0]
            nav_msg.origin.longitude = self.origin[1]

            self.displacement_ne = frame.geo2ne(self.point_ll, self.origin, self.geocentric_radius)

            # pose
            nav_msg.position.north = self.displacement_ne[0]
            nav_msg.position.east = self.displacement_ne[1]
            nav_msg.position.depth = 0
            # nav_msg.position.depth = -xsens_msg.position.altitude
            nav_msg.altitude = xsens_msg.position.altitude

            # TODO: simplify the conversion
            # IMU returns rotation from NWU in degrees
            # orient_xyz = np.array([xsens_msg.orientation_euler.x, xsens_msg.orientation_euler.y, xsens_msg.orientation_euler.z])
            # orient_xyz = np.deg2rad(orient_xyz)
            orient_ned = np.array([xsens_msg.orientation_euler.x, xsens_msg.orientation_euler.y, xsens_msg.orientation_euler.z])
            orient_ned = np.deg2rad(orient_ned)

            # Apply rotation to get from sensor_xyz to boat_xyz rotation
            # orient_xyz = frame.wrap_pi(orient_xyz - SENSOR_ANGLE_OFFSETS)
            orient_ned = frame.wrap_pi(orient_ned - SENSOR_ANGLE_OFFSETS)

            # Apply rotation to get from boat_xyz to boat_ned
            # orient_ned = frame.angle_xyz2ned(orient_xyz)

            # TODO: figure out why roll and pitch have to be inverted and no conversion from xyz to ned is necessary for both position and rate
            nav_msg.orientation.roll = -orient_ned[0]
            nav_msg.orientation.pitch = -orient_ned[1]
            nav_msg.orientation.yaw = orient_ned[2]

            # orientation rate is specified in XYZ frame
            orient_rate_xyz = np.array([xsens_msg.calibrated_gyroscope.x,
                                        xsens_msg.calibrated_gyroscope.y,
                                        xsens_msg.calibrated_gyroscope.z])

            orient_rate_ned = frame.angle_xyz2ned(orient_rate_xyz)

            # vel_ned is velocity of the sensor in NED Earth fixed reference frame
            vel_ned = np.array([xsens_msg.velocity.x, xsens_msg.velocity.y, xsens_msg.velocity.z])

            # apply a rotation to get from vel_ned to vel_body
            vel_body = frame.eta_world2body(vel_ned, orient_rate_ned, orient_ned)

            nav_msg.body_velocity.x = vel_body[0]
            nav_msg.body_velocity.y = vel_body[1]
            nav_msg.body_velocity.z = vel_body[2]
            nav_msg.orientation_rate.roll = -vel_body[3]
            nav_msg.orientation_rate.pitch = -vel_body[4]
            nav_msg.orientation_rate.yaw = vel_body[5]

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
        radius = frame.compute_geocentric_radius(point_ll[0])

        # NOTE: negative sign - that is because we want to look in the direction of origin
        self.origin = frame.ne2geo(-displacement_ne, point_ll, radius)
        # find the radius corresponding to the new origin
        self.geocentric_radius = frame.compute_geocentric_radius(self.origin[0])
        self.origin_set = True


if __name__ == '__main__':
    rospy.init_node('nav_new')
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


