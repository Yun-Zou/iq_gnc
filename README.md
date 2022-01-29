# Monash Motion
This package controls movement of the drone. This repo is a fork of the [iq_gnc package](https://github.com/Intelligent-Quads/iq_gnc) repo, which controls many of the low level control.[iq_tutorial](https://github.com/Intelligent-Quads/iq_tutorials) also has tutorials which can help you navigate this package.

The intelligent quads gnc_functions.hpp in /include are collection of high level functions to help make controlling your drone simple. You can find functions for interpreting commanding waypoints, changing modes and more. The documentation for using these functions is shown below. This function will communicate with mavros which will then communicate to the drone to perform the actions required.

[gnc_functions.hpp documentation](https://github.com/Intelligent-Quads/iq_tutorials/blob/master/docs/GNC_functions_documentation.md)

In /src, there are FlightAlgorithm.cpp, FlightController.cpp and FlightController.hpp files. FlightController class defines a set of flight modes and handles many of the topic and processing tasks. The FlightAlgorithm class will append the specific waypoints needed to produce circle/search/straight line movements into a waypoints array which will be called after the oldest waypoint has been reached.

Flight modes are defined in FlightController.hpp at the top of the file and correspond to specific behaviours, many of which are set by FlightAlgorithm class.

Flight plans are files that are launched which define movement and mission of the drone. They are an easy way for users to create a customised mission while being able to avoid all the low-level programming of movement and processing. These files are located in /src/flight_plans.

The original iq_gnc flight plans are in the /src/iq_gnc and /scripts folder to be looked at as a reference.


## Configs
In the /cfg folder there are is an indoors and outdoors config which will be read depending on the launch file you run. You can change these parameters without having to re-build the package. The parameters are read in the FlightAlgorithm and FlightController classes which will set how the movement is run. Descriptions of what the parameters do are in the files.

## How to Run
Flight plan launch files are in /launch and are split into indoor and outdoor launch files which will read the corresponding config file while running the same procedureo.

`
roscore
roslaunch monash_main t265_all_nodes.launch
roslaunch monash_motion xxx.launch (flight plan launch file)
`

## iq_gnc Original flight plans
### avoidance_sol.cpp
Example obstacle avoidance program utilizing the potential field method.

### gnc_tutorial.cpp
Simple waypoint mission that commands a drone to fly a square pattern. 

### sr_sol.cpp 
Simple search and rescue program that will fly a search pattern until yolo detects a person. This will trigger the drone to land to deliver the rescue supplies. 

### subscriber_sol.cpp
Example program showing how to use a ROS subscriber to take input into your drone's guidance node.