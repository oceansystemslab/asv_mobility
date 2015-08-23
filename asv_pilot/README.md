ASV pilot
=========

This module includes nodes for controlling the motion of the vehicle. It also contains 
the launch files required to run the vehicle (except for modem related functionalities)
and scripts that allow for control of the vehicle.

Nodes
=====

asv_pilot
---------

Takes position or velocity request as an input and with use of navigation data pilots the vehicle 
via sending ThrusterCommands to the pololu driver.

The controller will stop issuing motor requests if navigation information becomes old.

The controller is two layered PID. The gains of the PID controller can be updated in 
conf folder. The gains can be updated as the node is running provided that /src/pid_config is called.

Various position requests can be issued:
 - position_req - some North East position in NED frame with origin at navigation's origin
 - body_req - some North East position with respect to the vehicle's position
 - geo_req - longitude and latitude request

The pilot will not allow for sending vehicle to a point that is farther than 200m away.

The controller is disabled on start. It has to be enabled with a service call. Use script:

   ./scripts/enable_control.sh.

Note that this enables also pololu driver. To disable controller run:

   ./scripts/disable_control.sh

path_follower
-------------

Consumes PathRequest with a specified list of points, type of points and timeout of the path.
Issues NED position requests to the asv_pilot.

The points can be specified in NED frame with origin fixed at navigation's origin (changing nav's origin 
will terminate path!) or with GPS coordinates. Consider scripts corresponding to sending path requests:
 - gps_path_long.sh
 - gps_path_lawn.sh
 - ned_path_simple.sh
 
 To stop execution of current path use clear path service - clear_path.sh script. To start a new path
 previous path has to be completed or cleared.
 
 If a path times out a stay request is issued.

circler
-------
 
Node which can be used for following a circular path around some vehicle or around some static point.
 
thrusters_sim
-------------
 
Node for simulation of Emily's thrusters dynamics. Inputs and outputs are the same as pololu_driver.
