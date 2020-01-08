# Cyphy Testbed

## Overall Description
This repository represents a testing and development framework for application in the domain of cyberphysical systems. 

Currently, the "*Crazyflie*" drone by *Bitcraze* is the platform on which the test are run, but the system has been designed to include different platforms in the future. 
The core of the framework is based on *ROS*.


## Components
The framework has been organized in modules (*ROS packages*) for ease of maintenance.

Currently the following modules have been implemented:

1. **commander_interface**
This package provides the point of interaction between the vehicle and the user. The *commander\_node* advertises services for requesting task to the drone, such as goto, land, takeoff and full state tracking.
2. **controller**
This package contains the controllers to command the drone offboard.
3. **crazyflie_ros**
This is the package providing the services to communicate with the *Crazyflie* drone.
4. **demo_launchers**
This packages contains the *ROS* launch files to simplify the instantiation of the system components.
Currently, the main launchers are
    1. "cf\_bridge": Send a request to add a *Crazyflie* to the system and configure the vehicle. This requieres the presence of a crazyflie radio server active to take the request and start a communication thread.
    2. "radio_server": Start a node that provide services to add *Crazyflie*s to the system. It spawns a thread for each connected *Crazyflie*.
    3. "commander_launch": Starts the Commander node and the Guidance node.
    4. "controller_launch": Start the remote Controller node.
    5. "estimator_launch": Start the estimator node that aggregates the information provided by the MOCAP to estimate the pose of the vehicle.
    6. "vrpn_lauch": Starts the acquisition of data from the input sources, such as *Optitrack*. The launch file starts also an instance of *rViz* to visualize the vehicle/trajectory/estimation in a virtual environment.
    7. "demo_core": Start the datastream, the commander and arena parts of the framework.
    8. "ext_control": Starts the offboard controller node.
5. **guidance**
This package provides the guidance for the drone, that is, given a command and the current status of the drone, it generates the references to achieve the task. 
6. **monitors**
This package contains nodes to convert *ROS* topics into a form that can be represented in *rViz*, such as odometry.
7. **state_aggregator**
This package provides the node that is aggregating the external information about the drone (currently, just the Optitrack) and provides an estimate of the drone state.
8. **testbed_msgs**
This package contains the common custom messages used by the nodes.


## Setting up the framwork
The project was developed in *ROS Melodic* and some modules used *Eigen*. Make sure to have *ROS* and Eigen available on your system. 

Dependencies:
- [Eigen](https://eigen.tuxfamily.org) -- a header-only linear algebra library for C++
- [python-scipy] -- python library for math computation

The repository contains submodules, thus, make sure you clone *recursively*.
To build the framework, navigate to the `ws/` directory (the _workspace_), and run
```
catkin_make
```

Every time a new terminal is starged, it's necessary to reload the references to the framework packages. This is done, running the following command from the `ws/` directory:
```
source devel/setup.bash
```

## Basic Example
Basic test:
It is assumed that the Motive software is running, and that the rigid body associated with the drone is in the set {cf1, cf2, cf3}.

The commands to launch the necessary modules are:
```
roslaunch demo_launchers demo_core.launch
```
This start the core nodes: 
- vrpn
- estimators
- commander
- arena
- swarm manager

These commands should start the rViz visualization tool and the basic nodes. 

If the Optitrack is connected to the machine and its VRPN server is streaming rigid body data (connection parameters should be configured in the "vrpn_launch.launch file) it should be possible to visualize the drone frame moving in the rViz environment.

It is possible to test the commander calling the ros service to request a goto movement.
The argument for the service is a tuple of float32, representing the position [x, y, z], and a float32 representing the duration of the requested movement. 
For example: 
```
rosservice call /cf1/Commander_Node/goTo_srv '[1.0, 1.0, 1.0]' '3.0'
```
The rViz visualizer should plot a reference frame (ghost) moving towards the requested point. That movement of that frame is generated simulating the requested trajectory. 

## Control Example
To run a full example:
1) Launch the core of the framwork (state aggregator + guidance + arena node)
```
roslaunch demo_launchers demo_core.launch
```
This start the core nodes: 
- vrpn
- estimators
- commander
- arena
- swarm manager


2) Start the radio server to manage the communication with the *Crazyflie*:
```
roslaunch demo_launchers radio_server.launch
```

3) Launch the radio communication and the control nodes.
```
roslaunch demo_launchers controller_launch.launch vehicle_frame:=cf2 uri:=radio://0/100/2M/E7E7E7E7E7
```
The nodes started with the script are:
- cf\_server
- control\_node
- cmd\_vel\_converter


# Troubleshooting
Common problems are related to the communication between nodes.
1) Check that you are connected to the same network where the VRPN server is.
    Check in the "demo_core.launch" or the "datastream_launch.launch" for the IP address of the VRPN server.
2) Check that the name of the drone in the Motive software is the same used with the launch files
