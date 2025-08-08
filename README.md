# Multi UAVs Formation Control Based on ROS and PX4 

## Overview

ROS node files in `\src` aims to realize multi-UAVs formation control. Establish communication between ROS nodes, PX4 and Simulator through MAVROS. The version of PX4 firmwire is v1.16.0



The formation in this project contain $6$ UAVs ($1$ leader, $5$ followers). The followers' pose and velocity depend on the desired formation and the leader   

## 1. Task Description

This project can execute $4$ types of tasks for the UAV formation: initialization and takeoff, mission execution, return and land. Each task corresponds to an enum value 

### Initialization and Takeoff

Public the following commands in console to realize arming, offboard mode switch and takeoff for UAVs (wait a while patiently): 

`rostopic pub -1 /uavX/user std_msgs/Byte "data: 1"`

X denotes the UAV's index here ($X =0 \sim 4$)

![alt](/img/init1.png)

![alt](/img/init2.png)
<p align="center"> Initialization for control </p>   

![demo](/img/init and takeoff.gif)  
<p align="center"> Initialization and takeoff demo </p>  

(For demonstration, all videos has been sped up)

### Mission

Public the following commands to realize formation reconfiguration and formation flight to the goal point

`rostopic pub -1 /uavX/user std_msgs/Byte "data: 2"`

![demo](/img/mission.gif)  
<p align="center"> Mission demo </p>  

### Return

Public the following commands to realize formation return and all UAVs return to their home position

`rostopic pub -1 /uavX/user std_msgs/Byte "data: 3"`

![demo](/img/return.gif)  
<p align="center"> Return demo </p>  

### Land

Public the following commands to realize all UAVs land and be unarmed

`rostopic pub -1 /uavX/user std_msgs/Byte "data:4"`

![demo](/img/land.gif)  
<p align="center"> Land demo </p>  

## How to run
Follow [here](https://px-4.com/v1.14/en/sim_gazebo_classic/multi_vehicle_simulation_gazebo.html) for PX4 multi-sim setup

Firstly, launch `6_uav_mavros_sitl.launch` to configure the SITL environment

After that, launch `formation_6m_radius_6_uav.launch` to configure params for each UAV node, establish communication between the node and FCU and complete other preparations 

After formation controller setup is ready and FCU communication is established, you can publish `/uavX/user` commands for further tasks

## Reference

[matthewoots/formation_controller_ros](https://github.com/matthewoots/formation_controller_ros)
