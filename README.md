# COBPjs - TurtleBot3

This repository contains a project that implements the controls of a TurtleBot3 using COBPjs. 

This implementation is based on and uses code from the BPjs implementation of a TurtleBot simulation that can be found [here](https://github.com/bThink-BGU/RosBP/tree/COBP) and implements and extents its functionality with COBPjs.

This project uses [COBPjs](https://github.com/bThink-BGU/BPjs-Context) which is based on [BPjs](https://github.com/bThink-BGU/BPjs). A BPjs tutorial can be found [here](https://bpjs.readthedocs.io/en/latest/) and an explanation of COBP is provided by this [paper](https://www.sciencedirect.com/science/article/pii/S095058492030094X). To execute JavaScript code in Java, [Mozilla Rhino](https://github.com/mozilla/rhino) is used.

## Requirements
This project requires:
* [ROS with gazebo (tested with melodic)](http://wiki.ros.org/melodic/Installation)
* [ROS bridge](http://wiki.ros.org/rosbridge_suite/Tutorials/RunningRosbridge)
* [The TurtleBot3 packages](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/)
* [The TurtleBot3 simulation package](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#gazebo-simulation)


## How to run this project
To run this project:
* run in shell: roscore
* run in shell: export TURTLEBOT3_MODEL=waffle
* run in shell: roslaunch turtlebot3_gazebo turtlebot3_world.launch
* run in shell: roslaunch rosbridge_server rosbridge_websocket.launch 
* run the project

## Contents of this repository
This application mainly uses four files. The first two are located in the "java" folder, the first of which is the "Turtlebot3Main.java". This file contains the main method of the application which sets up and starts the COBPjs application. The second one is the "RosBridge.java"-file which manages the connection between the COBPjs application and the TurtleBot. 

The other two files are located in the resource folder and are the JavaScript files that contain the actual application code. 
These are the "cobp_sim.dal.js" and "cobp_sim.bl.js" which contain the data (context) aspects and the behavior aspects of the application respectively. Since this application has been incrementally implemented, the resource folder also contains subdirectories that contain previous stages of the program. The used files can be changed by altering the resource names and path in the constructor call of the ContextBProgram in the main file. Example: 

```java
BProgram bprog = new ContextBProgram("collision-avoidance/cobp_sim.dal.js", "collision-avoidance/cobp_sim.bl.js");
```

## Functionality
Upon the start of the program, the robot will start to perform the deliveries that are specified in its initial context. A delivery consists of a start and an end point, so the robot will first move to the coordinates of the start and then to those of the end. When the battery charge is low, it will halt its delivery and move to a specified charging point. Afterwards it will continue its deliveries. Once all deliveries are completed the robot will stop moving.

The communication between the application and the TurtleBot is realized by rosbridge. The ROS-topics that are used for communication are specified in the "cobp_sim.bl.js"-file and are "/scan" and "/odom" to receive information from the TurtleBot and "/cmd_vel" to send movement instructions to it. The rosbridge client uses WebSockets to communicate with the running rosbridge server which publishes and subscribes to the actual ROS-topics. When a message is received, its data is put into an external event which is then enqueued into the event queue of the application. An Live-Copy of a CBT can wait for those events and process their data. 
On the other hand, an event listener is used to react when a event with the name "UpdateVelocity" is selected and then publishes its movement data to the "/cmd_vel" topic. 

There are two additional topics that the application uses that are not necessary for the functionality of the application but can be used to interact with it. 

The first one is "/delivery_info" which is used to publish information about the current action of the robot. It can be used to help to understand what is currently happening in the application by echoing its messages. Example: 
```shell
rostopic echo /delivery_info
```

The second one is "/add_del" which can be used to add delivery orders to the robot. Example: 
```shell
rostopic pub --once /add_del geometry_msgs/PoseArray "{poses: [{position: {x: 0.0, y: -2.0}}, {position: {x: -2.0, y: 0.0}}]}"
```
The application was tested with the TurtleBot3 model waffle on the map turtlebot3_world. Theoretically other maps could also be used however the specified coordinates might need to be changed.  # COBP-rep-tes
# COBPjs-TurtleBot3
