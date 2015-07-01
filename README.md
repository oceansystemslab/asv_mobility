asv_mobility
============

This repo holds the ROS packages for Autonomous Emily boat. Some of the packages are vehicle specific (e.g. motor driver, thruster simulator or navigation).
Some of the tools are expected to be useful in other surface vehicles (asv_controllers, frame_maths).
It is important that this repo remains compatible with ROS Electric as this is the current version of ROS installed on Emily.
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
  - A valid ROS installation (version Electric or later is suggested)
  - Python 2.7+
  - Numpy 1.8+

TODOs:
-------------
  - Prepare launch files which allow for running Emily and Nessie at the same time.

Run Simulator
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
  rosservice call /pilot/switch "request: true"
  ```
  
4) Send a command to the pilot using the command-line (i.e. move the vehile to zero position):
  ```
  rostopic pub /pilot/position_req vehicle_interface/PilotRequest "position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"
  ```

Extra: For trajectory tracking, enable the path:
  ```
  roslaunch vehicle_core path_controller.launch
  asv_pilot/paths/pp_fast_long.sh
  asv_pilot/scripts/path_cli.sh start
  ```

Run Real Operation
---
1) Launch navigation and wait until GPS fix and convergence (leave on for a minute):
  ```
  roslaunch emily_nav nav_imu.launch
  ```

2) Launch pololu and pilot:
  ```
  roslaunch asv_pilot pololu_pilot.launch
  ```

3) Proceed as you would in simulation.

[semver]: http://semver.org/
[kiss]: http://en.wikipedia.org/wiki/KISS_principle
[tdd]: http://en.wikipedia.org/wiki/Test-driven_development
[solid]: http://en.wikipedia.org/wiki/SOLID_(object-oriented_design)
