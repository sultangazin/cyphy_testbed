# Cyphy Testbed (LFD branch)

## Overall Description
This repository represents a testing and development framework for application in the domain of cyberphysical systems. 

Currently, the "*Crazyflie*" drone by *Bitcraze* is the platform on which the test are run, but the system has been designed to include different platforms in the future. 
The core of the framework is based on *ROS*.

This particular instance of the repository has additional files to facilitate learning control from expert demonstrations.

## Components The framework has been organized in modules (*ROS packages*) for ease of maintenance.

Currently the following modules have been implemented:

1. **commander_interface**
This package provides the point of interaction between the vehicle and the user. The *commander\_node* advertises services for requesting task to the drone, such as goto, land, takeoff and full state tracking.
These requests are then managed by the Guidance that generates trajectories.
2. **control_router**
This package takes care of selecting the controller that will control the drone. Currently, it is possible to select among onboard or offboard controller and, in case of multiple offboard controllers, which one is actually driving the vehicle.
3. **crazyflie_controllers**
This package contains the controllers to command the drone offboard. In particular, it contains the learned control *lfd\_controller* that is based off of the expert controller contained in *geometric_controller*.
3. **crazyflie_ros**
This is the package providing the library and services to communicate with the *Crazyflie* drone.
4. **demo_launchers**
This packages contains the *ROS* launch files and scripts to simplify the instantiation of the system components.
Currently, the main launchers are
    1. "real_experiment": Launch file that starts all the components to run an experiment with the real drone.

    2. "simulation_experiment": Launch file that starts all the components to run the system in simulation. 
    
    3. "real_expertiment_LFD": Launch file that is exactly like "real_experiment", but uses "lfd_controller" to control the drone.
    
    4. "real_experiment_Geometric": Launch file that is exactly like "real_experiment", but uses "geometric_controller" to control the drone.
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
- [VRPN](sudo apt install ros-noetic-vrpn) -- install vrpn library
- [python3-scipy] (sudo apt install python3-scipy) -- python library for math computation
- [cvxopt] (sudo apt install python3-cvxopt) 
- [python-is-python3] (sudo apt install python-is-python3) -- python will then refer to python3
- [arena-py] (pip install arena-py) -- python library for the ARENA


The repository contains submodules, thus, make sure you clone *recursively*.
```
git clone --recursive <repoaddress>
```

To build the framework, navigate to the `ws/` directory (the _workspace_), and run
```
catkin_make
```

Every time a new terminal is started, it's necessary to reload the references to the framework packages. This is done, running the following command from the `ws/` directory:
```
source devel/setup.bash
```

## LFD Example

Prior to running the framework, run the simulation in MATLAB by running *sim\_expert.m* in *sim\_files*. This creates a file *K.csv*. Place it into *crazyflie\_controllers/lfd_controller/config/data/*.

The command to launch the framework with LFD controller is:
```
roslaunch demo_launchers real_experiment_LFD.launch
```
When launching the nodes, the system could request a login into the Arena. You can use the personal google account to perform the identification.

Once everything is started, the output of the system can be visualized at https://arenaxr.org/<yourusername>/LandOfGG.

To launch the script that commands the quadrotor to follow the figure eight, go to *ws/scripts/* and run *./figure_eight.sh*.
    
The command to launch the framework with the expert geometric controller is:
```
roslaunch demo_launchers real_expertiment_Geometric.launch
```

## Basic Example
Basic test simulation:

The commands to launch the system are:
```
roslaunch demo_launchers demo_anchors.launch
```
This launch file will start the nodes: 
- state_aggregator 
- drone simulator
- anchor_sim 

It is then possible to play with the system, such as changing the frequency of the simulator publications:
```
rosservice call /area0/sensors/anchors/anchorSimStatus "{freq: 40.0, active: true}"
```
or change the distortion of one anchor:
```
rosservice call /area0/sensors/anchors/anchorSimCtrl "{id: 0, enable: true, enable_distortion: true, distortion: -5.0}"
```


## ROS Control Example
Example with simulated vehicle, feedback linearization controller and Arena:

Before launching the system, set an environmental variable to select the scene on the arena server. From the terminal 
```
export SCENE='LandOfOz'
```

Then, launch the system with
```
roslaunch demo_launchers simulation_experiment.launch
```
This launch file will start the nodes: 
- state_aggregator 
- drone simulator
- feedback linearization controller
- control router 
- commander interface 
- guidance
- rarena

When launching the nodes, the system could request a login into the Arena. You can use the personal google account to perform the identification.

Once everything is started, the output of the system can be visualized at https://arenaxr.org/<yourusername>/LandOfOz

That scene is remote and it will be shared among all the participants. Being a collaborative environment, if you launch the experiment at the same time it will be a mess. 

It is possible to create new scenes if you want. The documentation for the Arena can be found at https://arena.conix.io/

In order to control the simulated drone, it is necessary to specify that the controller is an offboard one.
This can be done calling the service provided by the commander interface for the vehicle "cf2".
```
rosservice call /cf2/Commander_Node/ctrl_offboard_srv "offboard_active: true"
```

Once the offboard control is active, it is possible to issue goto commands to the drone using the services provided by the same commander node.

The argument for the service is a tuple of float32, representing the position [x, y, z], a float32 representing the duration of the requested movement and a boolean to select a "relative" or "absolute" motion. 
For example, to issue the drone cf2 a goto to [1.0, 1.0,  0.5] in 10.0 seconds relative to the current position: 
```
rosservice call /cf2/Commander_Node/goTo_srv '[1.0, 1.0, 0.5]' '10.0' 'false'
```

# Common problems
1. When running python nodes, check that the python file running the node is made executable: 
```
chmod +x <path_to_the_file>
```

2. In case you are connected to the network where the optitrack system is streaming information, but you don't receive anything, check the firewall to enable the incoming traffic from the optitrack machine.

