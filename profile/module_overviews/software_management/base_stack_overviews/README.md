# Base stack overview
This document describes various RAS project lines and the most common selection of modules to achieve certain vessel control behaviors. 

## Typical components
Minimally we need a couple of components to close a feedback loop. Commonly this can be divided in the following subtasks:
- A vessel (physical or simulated)
- Means of finding a vessel state (sensors, filters, state estimators)
- Means of affecting the vessel (actuators)
- Some agent that makes a decision (controller)
- Some objective of the system (e.g. task or reference)

There is not necessarily one module per function. It is possible that:
- One module covers multiple functions.
- A single module covers a single function.
- A function is covered by multiple modules.


This depends on the design choices and goal of a developer. Modularizing when possible enhances reusability, while doing various computational tasks within a single module can occasionally be simpler to implement.

### Robotic Operating System
We commonly use the Robotic Operating System (ROS) to facilitate communication between various modules. This allows various agents to stream (publish) and listen (subscribe) to any datastreams (topics) on the network to fulfull their task. Starting various control processes (nodes) within the network can together communicate and fulfull their tasks to close the feedback control loop. 

## An example of a simple Tito Neri waypoint following stack
### High level system view:
Here you can see an example of a simple setup that allows a ship to automatically follow a user defined set of waypoints. Waypoints are configured in the high level waypoint manager, that tracks whether the ship should aim at a next waypoint and streams a reference heading for the vessel. The heading control module converts reference and measured heading into actuator reference. The vessel has it's own low level control feedback loop of the actuators, imposing a force on the hull of the ship. The vessel has heading and geo position sensors providing a measured state for the waypoint manager and heading controller. 
In this case the connections between the 'vessel system', ' waypoint module' and 'heading controller' would be facilitated by ROS. <br> <br>
![wp_following vel_control schematic digipact 1 drawio (1)](https://user-images.githubusercontent.com/5917472/226966655-45b8debe-33da-4ca1-979a-b35bf5596c4a.svg)


### A variation
The point is that we can now use the previous example and other off-the-shelf components to make variations of the beforementioned control system. We can mix and match components to reach a desired behaviour / complexity matching a certain project/usecase. Below is an example where the system above has been adapted with besides heading control (with fixed motor rpm) to have velocity control based on a rectangular wave reference. <br> <br>
![wp_following vel_control schematic digipact 1 drawio](https://user-images.githubusercontent.com/5917472/226966073-ab6abd19-5175-4e65-82a9-91f2a4782680.svg)


### A detailed view on the vessel system
A detailed view on the devices on the vessel show some more complexity on board of the ship, shedding some light on the various software and hardware components on board. You can see that there are a variety of devices on board that run several software modules. There is a lot more going on than just a rigid body and some actuators, although the main functionality for external users is relatively simple: actuation reference goes in, sensor data comes out. Other control components can run on any devices on the ROS network. <br> <br>
![base_titoneri_setup drawio (1)](https://user-images.githubusercontent.com/5917472/225702533-7e9da585-7c21-45fe-a4b3-fe410db8400e.png)

## Other available modules

| Module name | Role | URL | Notes |
|-------------|------|-----|-------------------------|
| Matlab JoystickApp| Generating vessel actuation as human operator  |     |output compatible with Tito-Neri, Delfia |
|             |      |     |  |
|             |      |     |  |
|             |      |     |  |


## Mechatronics course base stack
Example function: Dynamic positioning with 
