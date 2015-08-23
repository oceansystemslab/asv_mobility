Emily Navigation package
========================

This node accepts message from MTi-G sensor drivers and generates a navigation message.

It is recommended that origin is set manually. This can be done with origin parameter. See nav launch files
in asv_pilot package for some examples.

If the origin chosen is far from the current position (farther than ~5km) nav will not be published for safety reasons.

Origin can be reset to current position with use of Boolean service on /nav/reset.

The node publishes also low frequency nav that can be routed through modem or used for vehicle monitoring without
putting much strain on the connection. The rate and topic can be set through params. 

The navigator can be used with various params:
 - wait_for_GPS - nav will not be published until GPS fix has been obtained - recommended for operation;
 set to false when testing navigation indoors
 - zero_pitch_roll - pitch and roll is forced to zero value in the nav - expected to smooth out the behaviour of the controller
