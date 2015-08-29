asv_mobility
============

This repo holds the ROS packages for Autonomous Emily boat. Some of the packages are vehicle specific (e.g. motor driver, thruster simulator or navigation).
Some of the tools are expected to be useful in other surface vehicles (asv_controllers, frame_maths).
It is important that this repo remains compatible with ROS Diamondback as this is the current version of ROS installed on Emily.
Many functionalities of the packages are available only if vehicle_core repository is accessible.

Guidelines
----------

Before attempting any modification to this repo please make sure thar:
  - you are working on an up-to-date version of the `master` branch
  - the previous working version of the repo has been tagged (`git tag --list` to check the tags)
  - you have considered the option of creating a new branch for your feature (`git checkout -b <branch_name>` to create a new branch), after all this is the recommended approach!
  - you know what are you doing!
  
Initially this repo is providing packages using the `rosbuild` build system until the OSL vehicles are migrated to an
 upgraded version of the ROS. Later the rosbuild support is going to be dropped and the master branch will offer a
 catkinized package format. The software development follows a couple of principles that have been proven useful
 during the early development of this project like [Semver][semver], for semantic versioning of releases and tags,
 [KISS][kiss], as a general guideline to prevent the commit of _huge_ files and _beast_ modules, and, finally,
 [TDD][tdd], as a principle for testing your code if you want to rely on a more pragmatic approach.

Requirements
------------
  - A valid ROS installation (version Diamondback or later is suggested)
  - Python 2.7+
  - Numpy 1.8+

General vehicle usage
====================

Connecting to Emily (WiFi on Emily side)
-----------------------------------------

Power up EmilyAP router, connect your computer with Ethernet cable to one of the ports on the router (for better bandwidth) or connect via WiFi - essid: EmilyAP. Make sure you have an IP address on network 192.168.42.X.

Connect USB WiFi adapter with an antenna to Emily's PC. Power it up (either via bench power supply or Emily batteries). Once Emily finishes booting process it will connect to the router via WiFi. Its IP address is 192.168.42.1.

The address of the router is 192.168.42.10. Once it is connected to the osl network Emily should have Internet connection.

At this point you should be able to ssh or telnet into Emily. Username is emily, password is seebyte.

Connecting to Emily (Ethernet)
-----------------------------------------

Power up EmilyAP router, connect your computer with Ethernet cable to one of the ports on the router (for better bandwidth) or connect via WiFi - essid: EmilyAP. Make sure you have an IP address on network 192.168.42.X.

Connect Ethernet cable to Emily's PC. Power Emily up (either via bench power supply or Emily batteries). Once Emily finishes booting process it will be available under 192.168.43.1 address. Give yourself an IP address on 192.168.43.X on Ethernet.

You can then ssh or telnet (look above). 

Running ROS with tmux
---------------------

The WiFi link to Emily is very intermittent. Do not run more than 2 ssh terminals at the same time, try to use only 1.

Learn how to use tmux. tmux allows you to open a terminal run some commands in it and then detach from it. This 
allows for keeping the ROS nodes alive even when network connection dies. For example:

First ssh to Emily:
  ```
  ssh emily@192.168.42.1
  ```
Then start tmux and run roscore in it:
  ```
  tmux
  roscore
  ```
Once started detach from the session by pressing Ctrl+b and then d.

Then start another node and detach:
  ```
  tmux
  roslaunch asv_pilot indoor_nav.launch
  Ctrl+b, d
  ```
You can come back to the previously detached tmux sessions. This will get you back to roscore terminal:
  ```
  tmux attach -t 0
  ```
You can rename the sessions if you like to keep the terminals in order.

For full all functionalities run:
  ```
  roscore
  roslaunch asv_pilot hwu_nav.launch
  roslaunch asv_pilot pololu_pilot.launch
  rosrun asv_pilot path_follower.py
  roslaunch modem_tools emily_packer.launch
  roslaunch evologics_driver emily_modem.launch
  ```
Hanged ssh or telnet connection
------------------------------

Sometimes ssh or telnet connection gets stuck. It may return after few minutes. The fastest way to get back the control
is logging in with the other mechanism (with telnet is ssh is hanged) and restart ssh or telnet service:
  ```
  sudo service xinitd restart  # for telnet
  ```
or
  ```
  sudo service ssh restart  # for ssh
  ```
  
Then try to login again.

Broken network connection
------------------------

Sometimes Emily loses connection to the EmilyAP WiFi. When this happens Emily will attempt to reconnect every minute. The script used for that can be seen in /home/emily/scripts/recon_wifi.sh

As Emily restarts the wireless interface ROS network should not use wlan interfaces for its IP.

Once Emily reconnects to the EmilyAP all previously run terminals should available via tmux attach.

Running simulator
-------------

1) Run emily with an AUV simulator (note that this models Emily is an underwater vehicle) with Emily's thruster simulator and controller:
  ```
  roslaunch asv_pilot sim.launch
  ```

2) Run visualization - rviz (UWSim unavailable yet)
  ```
  roslaunch asv_pilot rviz.launch
  ```
  
3) Enable the vehicle pilot (for safety the pilot is not sending thruster commands if not enabled by the user):
  ```
  ./asv_pilot/scripts/enable_control.sh
  ```
  
4) Send a command to the pilot using the command-line (i.e. move the vehile to zero position):
  ```
  rostopic pub /pilot/position_req vehicle_interface/PilotRequest "position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"
  ```

5) For trajectory tracking, send the path:
  ```
  rosrun asv_pilot path_follower.py
  ./asv_pilot/scripts/ned_path_simply.sh
  ```

For more browse launch files in asv_pilot package.


Modifications --- only on ''real emily'':
-- the nav is launched from the ''asv_mobility_old/emily_nav/xsens2nav.py''. After checking the /nav/nav_sts and /imu/xsens, differences between the value of the orientation in yaw was noticed.  There was an offest for yaw (in the nav at the imu-orientation-euler parameter was added 3.14: orient_ned[2]+=3.14). This was commented from the code. This is valid for placing the box containing the IMU sensor in the boat, so that the white cable ties match (the handle of the boat is facing the right side of the boat, glued up).
