force-torque-sensor
===================

A catkin package to read measurements from any ATI FT sensor via XML.

To clone the repository:
- cd ~/catkin_ws/src
- git clone https://github.com/CentroEPiaggio/force-torque-sensor.git
- git submodule sync
- git submodule update --init --recursive


ToDO:
 - [ ] Use force-torque-sensor controller from ros_control
 - [x] Create an URDF of the sensor to visualize the forces and frames
 - [ ] Create plugin to be used in simulation
