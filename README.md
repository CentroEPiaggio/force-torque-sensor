force-torque-sensor
===================

A catkin package to read measurements from any ATI FT sensor via XML

Done:
 - [x] Use force-torque-sensor controller from ros_control
 - [x] Create an URDF of the sensor to visualize the forces and frames
 - [x] Create plugin to be used in simulation
 - [x] Merge nano17 and gamma sensor 

Use in simulation:

1) on first terminal for nano17 ft sensor type:

roslaunch force_torque_sensor ft_simulation_display.launch use_gamma:=false 

or for gamma ft sensor type:

roslaunch force_torque_sensor ft_simulation_display.launch 


2) open a second terminal and type:

rqt

and add to rqt_plot the topic my_sensor/ft_sensor_topic/wrench/force my_sensor/ft_sensor_topic/wrench/torque

3) open a thierd terminal and type

roslaunch force_torque_sensor cylinder_spawner.launch