# Overview

The **VRU robot-based testing system** uses ROS2 as its middleware. The goal is to be able to plan and reproduce hazardous situations for testing the safety features of autonomous drive vehicles. 

## Architecture 

The overall flowchart of the current system is described below.

<p align="center">
  <img src="./src/doc/Architecture_1.png" width="800">
</p>

<p align="center">
  <img src="./src/doc/Architecture_2.png" width="800">
</p>

<p align="center">
  <img src="./src/doc/Architecture_3.png" width="800">
</p>
The rqt_graph is shown below.

<p align="center">
  <img src="./src/doc/rqt_.png" width="1600">
</p>

Notice that because of the time constraint in the project SafeCLAI. Not all the nodes are connected exactly as designed some direct connections may be used to shorten the development time. 
## Features

This system includes the following features:

- **Plan and Reproduce** The system can plan a path for each actor in the scene, and define its starting event based on some criteria such as time, actor position, etc. In addition, the system also allows users to save the configuration to a file and read it for easier reproduction.
- **Stop all the actors** The system can (automatically) stop all the actors when the scenario is not going as planned.
- **Check the Time to collision**  tbd.

## Setting up the project

### installing mavros
mavros is a package which acts as an intermidiary layer between the ros2 network and the pixhawk network. This needs to be installed. More detailed information can be found here: https://docs.px4.io/main/en/ros/mavros_installation.html

*BEWARE THE DOCUMENTATION ON THIS ASSUMES THAT WE USE AN OLDER VERSION OF ROS2.
So if possible just follow the commands bellow.*

These are the commands that you need to run in this order in the terminal install it:
```sh
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
```

```sh
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
```

```sh
sudo apt-get install protobuf-compiler libeigen3-dev libopencv-dev -y
```

```sh
sudo apt-get install ros-humble-mavros ros-humble-mavros-extras ros-humble-mavros-msgs
```

```sh
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash ./install_geographiclib_datasets.sh
```


### installing px4 sim
More information about this can be found here: https://docs.px4.io/main/en/sim_gazebo_classic/

These are the commands that you need to run in this order in the terminal install it:
```sh
cd /path/to/PX4-Autopilot
make px4_sitl gazebo-classic
```

### Note
You need to edit a variable in the sdf file of the robot model to get it to work properly (this is hopefully fixed at the end of our iteration):

Go to this directory: 
```
/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/r1_rover
```

edit the r1_rover.sdf file to set ```<enable_lockstep>false</enable_lockstep>``` to false

## Compilations
Compilation is quite simple just run the following command in the project directory:
```sh
colcon build
```

## Launch

The **VRU robot-based testing system**  starts with the default parameters with the following command.

Start the gazebo simulation. Run the following command in the PX4-Autopilot directory:
```sh
make px4_sitl gazebo-classic_r1_rover
```

Run the mavros in the project directory:
```sh
ros2 run mavros mavros_node --ros-args --param fcu_url:=udp://:14540@ -r tf:=/px4/tf  -r tf_static:=/px4/tf_static 
```

Launch the vru system. NOTE check if the source directories are correct.
```sh  
source ~/autoware/install/setup.bash
source ~/ros2_vru_system/install/setup.bash 
ros2 launch basic_mobile_robot vru_system.launch.py
```

## TODO:
Edit the parameters so that the robot may arm without checks: https://diydrones.com/forum/topics/can-t-disable-pre-arm-checks

check if the local pos is correctly published
finish up the code / fix up the old code

The pos messages that are sent by the old code are the same? This is how it should be, the same pos needs to be sent over and over untill its reached
shouldn't the pos msgs be sent all the time while the robot is in offboard mode?
the px4 need constant messages on the setpoint_position/local topic, these messages contain the DESIRED location, the px4 will drive to those positions by itself.
why does the robot not move to the desired position? message wrong? maybe look into wat that local position NED thing is about.

arms fine, goes to offboard mode fine. message headers? also check if the measured odom pos of the robot matches the z value of the message, maybe its trying to fly or something? what does the 'guided' state mean?

try just publishing to the setpoint topic in a loop at 3 hz via the commandline. 
Airframes messed up? redo it?
vru in the sim?

must be an issue in the simulator. The issue is with the simulator.

put the sim up in a repo too.

x and y are mixed up in the GUI?

fail safe active?

try with raw local pos?

errors on start up, issue? doubt it.

check rqt, see what listens to setpoints.




```sh
ros2 topic pub /mavros/setpoint_position/local geometry_msgs/msg/PoseStamped "{
  header: {
    frame_id: 'odom'
  },
  pose: {
    position: { x: 20.0, y: 30.0, z: 1.0 },
    orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 }
  }
}" --rate 12

```

The code does not work properly when using the physical px4, when using the sim it works fine? 
Also when doing the following, it works fine for some reason? why?:
1. start mavros
2. start sim
3. start code
4. stop sim and set mavros to usb port
5. use the code
