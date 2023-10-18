# EGH400-2
A private repository for maintaining the code created within EGH400-2

This repository contains the code generated for providing control over simulated UAVs within the Gazebo flight simulator. The code from within this repository should be run from a linux-based machien with the required ROS2 Iron, PX4 and MicroXRCE DDS Agent software installed. Additionally, the terminal form which this code is run should have ROS2 Iron sourced within it before use.

## Required External Software Installation
The required external software noted above can be installed by following the instructions in the following documentation:

For PX4 and MicroXRCE DDS Agent installation:
- https://docs.px4.io/main/en/ros/ros2_comm.html

For ROS2 Iron installation:
- https://docs.ros.org/en/iron/Installation/Ubuntu-Install-Debians.html

## Running the simulation
Before running the ROS2 package responsible for providing control to the UAVs for flight planning, it is necessary to launch both a Gazebo simulation with the desired UAV configrations, and the Micro XRCE DDS Agent middleware

### PX4/Gazebo Simulator
Within a command terminal, traverse to the folder containing the PX$ installation, if the instructions within the provide documentation where used, this can be achieved through the command `cd ~/PX4-Autopilot`.

A single quadcoipter UAV can the be initialised with the gazebo simulator through the command:
```
PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL=x500 ./build/px4_sitl_default/bin/px4
```

To specify the initial location and unique ID of the created simulation vehicle, which is necessary for multi-vehicle simulaitons, the following command should be utilised:
```
PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL=x500 PX4_GZ_MODEL_POSE="<vehicle position>" ./build/px4_sitl_default/bin/px4 -i <instance>
```
Where the vehicle position is a comma spereated string of the desired initial vehicle position and attitude, and the vehicle instance is a positive integer value.

### MicroXRCE Middleware
Within a new terminal, the MicroXRCE DDS Agent middleware should be executed to allow for communicate between the created ROS2 package and the PX4 Autopilot within the Gazebo simulation. This can be achieved by moving to the source directory of the middleware, `~/Micro-XRCE-DDS-Agent` before running the agent:
```
MicroXRCEAgent udp4 -p 8888
```
