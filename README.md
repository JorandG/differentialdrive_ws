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
