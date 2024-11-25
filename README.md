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

## Launch

The **VRU robot-based testing system**  starts with the default parameters with the following command. note that autoware.universe is required to be installed first. 


```sh  
source ~/autoware/install/setup.bash
source ~/ros2_vru_system/install/setup.bash 
ros2 launch basic_mobile_robot vru_system.launch.py
```
