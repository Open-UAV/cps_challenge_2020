# Scripts to control drone in PX4 Gazebo environment

Look at `docking_demo.py` for code to attach and detach an object with a PX4 drone.
This script will also set mode to Offboard and arm the vehicle.

This script publishes to attach and detach commands, which require the following repository code to be setup and running.
https://github.com/Open-UAV/gazebo_ros_link_attacher 



Run `multi_vehicle_position_control_demo.py` in catkin_ws/src/cps_challenge_2020/scripts folder to move the drone through a series of position setpoints illustrating the key tasks for Phase II mission scenario. The rover can also be switched to offboard and armed for it to travel north with an average speed of 1 m/s. 
This script will need you manually set the mode to offboard and arm the vehicle.
Commands to set mode and arm the vehicle.
```
rosservice call /mavros/set_mode "base_mode: 0 custom_mode: 'OFFBOARD'"
rosservice call /mavros/cmd/arming "value: true"
```
 	
  
