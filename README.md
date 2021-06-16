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
These request are then managed by the Guidance that generates trajectories.
2. **control_router**
This package takes care of selecting the controller that will control the drone. Currently, it is possible to select among onboard or offboard controller and, in case of multiple offboard controllers, which one is actually driving the vehicle.
3. **crazyflie_controllers**
This package contains the controllers to command the drone offboard.
3. **crazyflie_ros**
This is the package providing the library and services to communicate with the *Crazyflie* drone.
4. **demo_launchers**
This packages contains the *ROS* launch files and scripts to simplify the instantiation of the system components.
Currently, the main launchers are
    1. "real_experiment": Launch file that starts all the components to run an experiment with the real drone.

    2. "simulation_experiment": Launch file that starts all the components to run the system in simulation. 
5. **guidance**
This package provides the guidance for the drone, that is, given a command and the current status of the drone, it generates the references to achieve the task. 
6. **rarena**
This package contains the nodes for the implementation of the ROS/Arena bridge. [ONGOING WORK]
7. **state_aggregator**
This package provides the node that is aggregating the external information about the drone (currently, just the Optitrack) and provides an estimate of the drone state.
8. **testbed_msgs**
This package contains the common custom messages used by the nodes.
9. **utilities**
This package contains helpers for the framework.
10. **vrpn_client_cyphy**
Module of the vrpn client for ROS.


## Setting up the framwork
The project was developed in *ROS Melodic* and some modules used *Eigen*. Make sure to have *ROS* and Eigen available on your system. 

Dependencies:
- [Eigen](https://eigen.tuxfamily.org) -- a header-only linear algebra library for C++
- [libusb-1.0-0-dev] (sudo apt install libusb-1.0-0-dev)
- [VRPN](https://github.com/vrpn/vrpn.git) -- install vrpn library from source and copy the FindX.cmake to the local cmake Module folder.
- [python3-scipy] (sudo apt install python3-scipy) -- python library for math computation
- [cvxopt] (sudo apt install python3-cvxopt) 
- [python-is-python3] (sudo apt install python-is-python3) -- python will then refer to python3
- [arena-py] (pip install arena-py) -- python library for the ARENA


The repository contains submodules, thus, make sure you clone *recursively*.
To build the framework, navigate to the `ws/` directory (the _workspace_), and run
```
catkin_make
```

Every time a new terminal is started, it's necessary to reload the references to the framework packages. This is done, running the following command from the `ws/` directory:
```
source devel/setup.bash
```
It's possible to avoid doing this by putting the previous command in the ~/.bashrc script.

## Basic Example
Basic test (simulation):

The commands to launch the necessary modules are:
```
roslaunch demo_launchers simulation_experiment.launch
```
This launch file will start the core nodes: 
- vrpn client
- estimator
- commander
- controller
- drone simulator
- arena bridge 

First it is necessary to specify that the controller is an offboard one with the command:
```
rosservice call /cf2/Commander_Node/ctrl_offboard_srv "offboard_active: true"
```

It is possible to test the commander calling the ros service to request a goto movement for a specific drone.
The argument for the service is a tuple of float32, representing the position [x, y, z], and a float32 representing the duration of the requested movement. 
For example, to issua the drone cf2 a goto to [1.0, 1.0,  0.5] in 10.0 seconds relative to the current position: 
```
rosservice call /cf2/Commander_Node/goTo_srv '[1.0, 1.0, 0.5]' '10.0' 'false'
```


## ROS Control Example
[TO BE FILLED]

# Troubleshooting
Usually, common problems are related to the communication between nodes.
1) Check that you are connected to the same network where the VRPN server is.
    Check in the "demo_core.launch" or the "datastream_launch.launch" for the IP address of the VRPN server.
2) Check that the name of the drone in the Motive software is the same used with the launch files
