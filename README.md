To generate the custom messages, do a catkin_make first of the workspace:

```
cd ~/differentialdrive_ws
catkin_make
```

Then use the following command in matlab:

```
rosgenmsg('~/differentialdrive_ws/src')
```

NB: You might need to delete the previous folder matlab_msg_gen_ros1 before using the command.

Then copy the the path given in the instructions after the command into the matlab code you want to use:

```
addpath('/home/jorand/differentialdrive_ws/src/matlab_msg_gen_ros1/glnxa64/install/m')
savepath
```

In order to use rosotpic echo of the topics /human_robot_interaction%i and /MILPResults%i you need to navigate to ~/differentialdrive_ws and source the setup.bash:

```
cd ~/differentialdrive_ws
source devel/setup.bash
```

```
rostopic echo /human_robot_interaction1
```

# To start the simulation use:

```
roslaunch diff_drive_robot gazebo_diff_drive_robot.launch
```

And to start moving the robot (after starting the HRJournal) use:

```
 roslaunch main_loop_pkg main_loop.launch 
```

NB: In the file parameters.yaml use "simulation: true" to use the simulation. You can also change the Odometry and Velocities topic for simulation or real world testing. The timing can also be set to "external: true" to use the Timing.py node.


# To start the real world testing:
First source the bashrc:
```
source ~/.bashrc
```

Where the IPs should be coherent with the network for example for the master:
```
export ROS_HOSTNAME=192.168.0.137
export ROS_MASTER_URI=http://192.168.0.137:11311
```

And for a turtlebot:
```
export ROS_HOSTNAME=192.168.0.185
export ROS_MASTER_URI=http://192.168.0.137:11311
```

Start the vrpn_client:
```
roslaunch vrpn_client_ros sample.launch 
```

To start moving the robot (after starting the HRJournal) use on the master PC:

```
 roslaunch main_loop_pkg main_loop.launch 
```
