Pololu driver
=============

This module is responsible for managing thruster and steering servo on Emily via PWM signal generation.

pololu_node accepts ThrusterCommand message and sets motors to desired state. If new messages 
do not arrive for a while the motors are set to default neutral states. If the message that arrived
is old it is ignored.

pololu_driver includes some low level bit manipulation required to set the motors to desired throttle/position.
