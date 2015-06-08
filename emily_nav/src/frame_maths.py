#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division
import numpy as np

# Constants
A_EARTH = 6378137.0  # metres - semi-major axis - assumes elipsoidal model of Earth
B_EARTH = 6356752.3  # metres - semi-minor axis - assumes elipsoidal model of Earth
SENSOR_ORIENT = np.array([np.pi, 0, np.pi])

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


def compute_jacobian(phi, theta, psi):
    """This functions computes the jacobian matrix used for converting body-frame to earth-frame coordinates.

    :param phi: pitch angle (k)
    :param theta: roll angle (m)
    :param psi: yaw angle (n)
    :return: J matrix (6x6)
    """
    J = np.zeros((6,6))

    # jacobian one
    J[0, 0] = np.cos(theta) * np.cos(psi)
    J[0, 1] = np.cos(psi) * np.sin(theta) * np.sin(phi) - np.sin(psi) * np.cos(phi)
    J[0, 2] = np.sin(psi) * np.sin(phi) + np.cos(psi) * np.cos(phi) * np.sin(theta)

    J[1, 0] = np.cos(theta) * np.sin(psi)
    J[1, 1] = np.cos(psi) * np.cos(phi) + np.sin(phi) * np.sin(theta) * np.sin(psi)
    J[1, 2] = np.sin(psi) * np.sin(theta) * np.cos(phi) - np.cos(psi) * np.sin(phi)

    J[2, 0] = -np.sin(theta)
    J[2, 1] = np.cos(theta) * np.sin(phi)
    J[2, 2] = np.cos(theta) * np.cos(phi)

    # jacobian two
    J[3, 3] = 1.0
    J[3, 4] = np.sin(phi) * np.tan(theta)
    J[3, 5] = np.cos(phi) * np.tan(theta)

    J[4, 3] = 0.0
    J[4, 4] = np.cos(phi)
    J[4, 5] = -np.sin(phi)

    J[5, 3] = 0.0
    J[5, 4] = np.sin(phi) / np.cos(theta)
    J[5, 5] = np.cos(phi) / np.cos(theta)

    return J


#pythran export update_jacobian(float[][], float, float, float)
def update_jacobian(J, phi, theta, psi):
    """This functions computes the jacobian matrix used for converting body-frame to earth-frame coordinates.

    :param J: jacobian matrix (6x6)
    :param phi: pitch angle (k)
    :param theta: roll angle (m)
    :param psi: yaw angle (n)
    :return: J matrix (6x6)
    """
    J = np.zeros((6,6))

    # jacobian one
    J[0, 0] = np.cos(theta) * np.cos(psi)
    J[0, 1] = np.cos(psi) * np.sin(theta) * np.sin(phi) - np.sin(psi) * np.cos(phi)
    J[0, 2] = np.sin(psi) * np.sin(phi) + np.cos(psi) * np.cos(phi) * np.sin(theta)

    J[1, 0] = np.cos(theta) * np.sin(psi)
    J[1, 1] = np.cos(psi) * np.cos(phi) + np.sin(phi) * np.sin(theta) * np.sin(psi)
    J[1, 2] = np.sin(psi) * np.sin(theta) * np.cos(phi) - np.cos(psi) * np.sin(phi)

    J[2, 0] = -np.sin(theta)
    J[2, 1] = np.cos(theta) * np.sin(phi)
    J[2, 2] = np.cos(theta) * np.cos(phi)

    # jacobian two
    J[3, 3] = 1.0
    J[3, 4] = np.sin(phi) * np.tan(theta)
    J[3, 5] = np.cos(phi) * np.tan(theta)

    J[4, 3] = 0.0
    J[4, 4] = np.cos(phi)
    J[4, 5] = -np.sin(phi)

    J[5, 3] = 0.0
    J[5, 4] = np.sin(phi) / np.cos(theta)
    J[5, 5] = np.cos(phi) / np.cos(theta)

    return J


ORIENT_NED_IN_XYZ = np.array([np.pi, 0, 0])
J_NED_IN_XYZ = compute_jacobian(*ORIENT_NED_IN_XYZ)
J_NED_IN_XYZ_INV = np.linalg.pinv(J_NED_IN_XYZ)
def angle_xyz2ned(orient_vehicle_xyz):
    """Converts an angle expressed in xyz frame to ned frame.

    :param orient_vehicle_xyz: [roll pitch theta] in radians
    :return: [roll pitch theta] in radians in ned frame
    """
    vehicle_orient_ned = np.dot(J_NED_IN_XYZ_INV[3:6, 3:6], orient_vehicle_xyz)
    return vehicle_orient_ned


def eta_world2body(vel_ned, orient_rate_ned, vehicle_orient_ned):
    """Converts velocity and orientation rate in world frame to body frame.

    :param vel_ned: linear velocities in world ned frame
    :param orient_rate_ned: angular velocities in world ned frame in radians
    :param vehicle_orient_ned: orientation of the body with respect to the world frame
    :return: velocities in body frame
    """
    vel_ned_full = np.concatenate([vel_ned, orient_rate_ned])

    J = compute_jacobian(*vehicle_orient_ned)
    J_inv = np.linalg.pinv(J)

    vel_body = np.dot(J_inv, vel_ned_full)
    return vel_body


def wrap_pi(angle):
    return ((angle + np.pi) % (2*np.pi)) - np.pi


if __name__ == "__main__":
    # rotations demo
    sensor_read = np.array([np.pi, 0, np.pi])
    print "Sensor reading: ", sensor_read
    veh_orient_xyz = sensor_read - SENSOR_ORIENT
    print "Vehicle orient in xyz: ", veh_orient_xyz
    veh_orient_ned = angle_xyz2ned(veh_orient_xyz)
    print "Vehicle orient in ned: ", veh_orient_ned, '\n'

    sensor_read = np.array([np.pi, 0, np.pi+0.1])
    print "Sensor reading: ", sensor_read
    veh_orient_xyz = sensor_read - SENSOR_ORIENT
    print "Vehicle orient in xyz: ", veh_orient_xyz
    veh_orient_ned = angle_xyz2ned(veh_orient_xyz)
    print "Vehicle orient in ned: ", veh_orient_ned, '\n'

