# RAS control modules
<p align="center">
  <img src="https://user-images.githubusercontent.com/5917472/225625088-a7cb7d74-f594-4aed-bfc7-555cc0f2d2e4.png" />
</p>

This section provides a description of the various maritime robotics experiment tools at your disposal. It serves as a starting point for collaborators to familiarize themselves with the available resources.The benefit of using the modules developed by RAS engineers is that these elements have usually been developed up to a reasonable state of reliability, which becomes extra desirable when there are numerous single-points-of-failure. Leveraging well-tested modules allows you to focus more on your goal. Utilizing proven components you can devote more time and effort to pushing your own project goals, while contributing by maturing the software through making small improvements, bug reports or feedback. 

If you intend to develop a control architecture utilizing any of these modules, we highly recommend sparring with the engineers of RAS when you want to start designing a control architecture.
<p align="center">
  <img src="https://user-images.githubusercontent.com/5917472/225585663-8054c647-83d7-4b90-abb3-9ec3994ab30f.png"/>
</p>

## Robotic operating system
Our design philosophy centers around creating reusable software components for future projects, where the [Robotic Operating System](https://www.ros.org/) facilitates this. The main functionality revolves around a modular publisher-subscriber communication structure, although it also supports in deployment, integration of external tools, data collection and makes valuable tools available. Most information flows between modules described below are utilizing ROS inputs and outputs. 

## Modules
The organisation repositories offer a variety of repositories that cater to researchers and projects. To ensure ease of use for upcoming projects, we organize these software components into different function groups.

### Vessels, Sensors & Actuators
| Name                | Description                                                                    |
| ------------------- | ------------------------------------------------------------------------------ |
| [Tito Neri actuator stack V3](https://github.com/RAS-Delft/RAS_TitoNeri/tree/main/ras_low_level_bridge)  |  Subscribes to reference actuation & executes it, using an ARM Microprocessor with PID feedback control. Publishes telemetry|
| Delfia actuator stack| Subscribes to reference actuation & executes it, using a microcontroller with PID Feedback. Publishes telemetry. (currently saved local on Delfia (1-3) Raspberry Pi's)|
| [GNSS ROS bridge ](https://github.com/RAS-Delft/reach_bridge)| Publishes differential gnss output on ROS|
| [Optitrack ROS bridge](https://github.com/RAS-Delft/ros_optitrack_bridge) | Publishes rigid bodies (e.g. vessel) poses measured on optitrack-equppped inside locations ROS |
| Pico-Logger | Voltage/current logging utility (in development) |

### Visualisation
| Name                | Description                                                                    |
| ------------------- | ------------------------------------------------------------------------------ |
| [Ship web diagnostics](https://github.com/RAS-Delft/web-diagnostics) | Webbrowser diagnostics and map state/reference display for robots connected to VPN network |
| [Rviz2 fleet visualisation](https://github.com/RAS-Delft/ras_urdf_common/blob/main/launch/rviz_bringup.launch.py) | Shows ships in a 3d environment using Rviz2. Requires robot description ('urdf' file in same repo) to be streamed for respective vessels. |
| [Plotjuggler](https://github.com/facontidavide/PlotJuggler) (not a ras repo, yet awesome) | General real time plotting of everything ROS. |

### General control modules
| Name                | Description                                                                    |
| ------------------- | ------------------------------------------------------------------------------ |
| [Joystick controller (Matlab Gui) ](https://github.com/RAS-Delft/RAS_General/blob/main/matlab/GUI%20%26%20control%20subsystems/Joystick_app_directJoyControl.mlapp)| Allows joystick control of actuators by a human operator|
| Control effort allocator 3dof  | Allocates a desired control effort of a delfia with simple constraints.   |
| Heading controller| Controls heading of a vessel (PID based) |link  |
| Waypoint following manager | Manages waypoints of a track that a vessel needs to follow. Has a click point user interface to adjust path. Calculates reference heading of a ship and detects if ship needs to snap to a next waypoint. |
| USV_geopos_heading_differentiator | Estimates ship body fixed velocities through differentiation and filtering |
| Surge velocity controller | Aims to let ship surge follow a reference (PID based) |
| Surge-yaw TitoNeri thrust allocator | Allocates desired control effort (Fx, Mzz) over thrusters |



### Project specific modules
| Name                | Description                                                                    |
| ------------------- | ------------------------------------------------------------------------------ |
| Mechatronics  course Simulink Tito Neri Dynamic Positioning | Dynamic positioning system for Tito Neri running on Matlab Simulink. |
| USV_formation_control_1| Control stack using multiple ships following a fixed speed moving formation reference along a predefined path. Ships employ heading and velocity control to steer towards and remain at desired distance from formation targets|










### Basic Building Blocks:
These are common software modules doing basic operations fundamental for reliable vessel operation. While they may not be of direct interest to regular researchers, they play a crucial role in the smooth functioning of the system. Examples of such modules include:
- Bridges between sensors and ROS (Robot Operating System)
- Low level actuator controls
- VPN services for secure communication
- Remote control abilities
- Ship diagnostics for monitoring and maintenance purposes

### Specific Use Case Components:
Some software components are tailored for specific subjects or applications. These components are bundled together in dedicated packages for easy access and usage. Examples of such packages include:
- Waypoint following demonstration for path planning and navigation
- Formation control system 1, which includes heading and distance keeping approaches (ref), ideal for formations of vessels
- MT44000 Mechatronics course software, catering to the specific needs of the Mechatronics course.


### Utilities 
| Name                | Description                                                                    |
| ------------------- | ------------------------------------------------------------------------------ |

| ras_tf_lib          | Various python functions packed in a ros library that is used in other modules |
| Nausbot             | Turtlebot but as a ship: lightweight real time ship simulator on a ros node    |


### Control modules
| Name                | Description                                                                    | Link              |
| ---------------- | --------------------- | ------------------------------ |
link  |
| DP + vessel assembly + formation_adaptive_control  | Vessel dynamic positioning, assembling and adjusting their 1)control effort generation and 2) allocation.  | |


### Backend systems
| Name                | Description                                                                    | Link              |
| ---------------- | --------------------- | ------------------------------ |
| Ros optitrack bridge | Takes optitrack body pose and streams it on ros |  |
| Web-Diagnostics | Python server and Vue.js webapp code that form the web diagnostics panel. |  |

| Server-Docker-Stack | Stack that runs VPN server services |  |
| boat-daemon | Sends vessel diagnostics to Web-Diagnostics |  |
| reach-bridge | Connects the emlid reach to stream vessel position on ROS |  |
| TN_arduino_low_level | Controls actuators to follow input reference and returns telemetry |  |
| ROS-Arduino-Bridge | passes Ros vessel actuator commands to microcontroller. Returns telemetry & Heading. returns system diagnostics |  |

## Interfaces
### Namespaces 
The table below shows identifiers are used for our vessels. These names are commonly used when something needs to refer to a specific ship, such as ROS topics. 

| Model      | Description          | Identifier  |
|------------|----------------------|-------------|
| TitoNeri   | Light-blue Tito Neri | RAS\_TN\_LB |
| TitoNeri   | Dark-blue Tito Neri  | RAS\_TN\_DB |
| TitoNeri   | Red Tito Neri        | RAS\_TN\_RE |
| TitoNeri   | Yellow Tito Neri     | RAS\_TN\_YE |
| TitoNeri   | Purple Tito Neri     | RAS\_TN\_PU |
| TitoNeri   | Green Tito Neri      | RAS\_TN\_GR |
| TitoNeri   | Orange Tito Neri     | RAS\_TN\_OR |
| GreySeabax | The Grey-seabax      | RAS\_GS     |
| Delfia-1*  | Delfia 1             | RAS\_DF\_1  |
| Delfia-1*  | Delfia 2             | RAS\_DF\_2  |
| Delfia-1*  | Delfia 3             | RAS\_DF\_3  |
| Delfia-1*  | Delfia 4             | RAS\_DF\_4  |
| Delfia-1*  | Delfia 5             | RAS\_DF\_5  |

### ROS topics & message formats
To have modules easily swappable and connectable, the following default messagetypes and namespacing are suggested. 'vesselID' refers to the Identifier of a particular vessel as described in the [previous section](#namespaces).

| Topicname                                 | Description             | Messagetype        | Default unit(s)                                        |
|-------------------------------------------|-------------------------|--------------------|--------------------------------------------------------|
| /vesselID/state/actuation | Measured actuator state | std_msgs/Float32MultiArray  | (see [below](#details-on-particular-topics) |
| /vesselID/reference/actuation  |  Reference/desired Actuator state     | std_msgs/Float32MultiArray* | (see [below](#details-on-particular-topics) )  |
| /vesselID/reference/actuation_prio  |  Override actuation reference, commonly for emergency | std_msgs/Float32MultiArray* | (see [below](#details-on-particular-topics) )   |
| /vesselID/state/pose_local | Estimated/measured pose w.r.t. local coordinate system    | geometry_msgs/Pose       | meters,quaternions    |
| /vesselID/reference/pose_local | Reference/desired pose w.r.t. local coordinate system    | geometry_msgs/Pose       | meters,quaternions    |
| /vesselID/state/geopos | Estimated/measured global position | sensor_msgs/NavSatFix      | degrees (latitude, longitude), meters (altitude)   |
| /vesselID/reference/geopos | Reference/desired global position | sensor_msgs/NavSatFix      | degrees (latitude, longitude), meters (altitude)   |
| /vesselID/state/yaw | Estimated/measured yaw | std_msgs/Float32 | radians (clockwise from above w.r.t. north) |
| /vesselID/reference/yaw | Reference/desired yaw | std_msgs/Float32 | radians (clockwise from above w.r.t. north) |
| /vesselID/state/actuator_forces | current forces/torques applied to ship in CO | geometry_msgs/Wrench | Newton (linear), Newton*meter (angular) |
| /vesselID/reference/actuator_forces | desired forces/torques applied to ship in CO | geometry_msgs/Wrench | Newton (linear), Newton*meter (angular) |
| /vesselID/state/velocity | measured/estimated velocities of CO in body fixed coordinate system | geometry_msgs/Twist | m/s (linear), rad/s (angular) |
| /vesselID/reference/velocity | desired velocities of CO in body fixed coordinate system | geometry_msgs/Twist | m/s (linear), rad/s (angular) |

#### Details on particular topics
For the Tito Neri the actuation message of JointState is characterized by a variable amount of actuator_name and value fields. The order of the joints does not matter, as long as the name array is ordered similar to the values. The values are then looked up by name. Thrust angles are communicated on the Position field. aft and propeller velocities on the Velocity field. Other data (e.g.  can be ignored or set to NaN. 

| Joint name                           | Joint name                 | Unit               | Data stored in field | Characteristic values |
|--------------------------------------|----------------------------|--------------------|----------------------|-----------------------|
| Aft portside propeller velocity      | SB_aft_thruster_propeller  | rounds per minute  | Velocity             | -2400:2400 |
| Aft starboardside propeller velocity | PS_aft_thruster_propeller  | rounds per minute  | Velocity             | -2400:2400 |
| Bow thruster velocity                | BOW_thruster_propeller     | Normalized         | Velocity             | -1:1 |
| Aft portside azimuth angle           | SB_aft_thruster_joint      | Radians            | Position             | -0.75pi:0.75pi        |
| Aft portside azimuth angle           | PS_aft_thruster_joint      | Radians            | Position             | -0.75pi:0.75pi        |  

<br>

Delfia-1* actuation array is as follows:
| Index | Content                  | Unit    |
|-------|--------------------------|---------|
| 0     | Rear propeller velocity  | RPS     |
| 1     | Front propeller velocity | RPS     |
| 2     | Angle rear azimuth       | Radians |
| 3     | Angle front azimuth      | Radians |


<br>
For the Tito Neri the telemetry array is defined as: (scheduled to be updated in 2024)

| Index | Content                   | Unit                                 |
|-------|---------------------------|--------------------------------------|
| 0     | dcmotor\_P\_reference     | RPM                                  |
| 1     | dcmotor\_SB\_reference    | RPM                                  |
| 2     | output\_BOW               | float, normalized on range -1.0:1.0) |
| 3     | reference\_Angle\_P       | deg                                  |
| 4     | reference\_Angle\_SB      | deg                                  |
| 5     | dcmotor\_P\_estimated     | RPM                                  |
| 6     | dcmotor\_SB\_estimated    | RPM                                  |
| 7     | dcmotorPID\_P\_Output     | normalized on 8 bit reslution 0-255  |
| 8     | dcmotorPID\_SB\_Output    | normalized on 8 bit reslution 0-255  |
| 9    | arduinoInternalClock      | ms                                   |
| 10    | IMU\_accell\_x            | m/s/s                                |
| 11    | IMU\_accell\_y            | m/s/s                                |
| 12    | IMU\_accell\_z            | m/s/s                                |
| 13    | IMU\_gyro\_x              | rad/s                                |
| 14    | IMU\_gyro\_y              | rad/s                                |
| 15    | IMU\_gyro\_z              | rad/s                                |
| 16    | IMU\_magneto\_x           | uT                                   |
| 17    | IMU\_magneto\_y           | uT                                   |
| 18    | IMU\_magneto\_z           | uT                                   |
| 19    | motor\_ESC\_SignalOut\_P  | 1200-1800Hz                          |
| 20    | motor\_ESC\_SignalOut\_SB | 1200-1800Hz                          |

### Coordinate systems
Various basins are equipped with systems that can find and stream state of rigid bodies, such as the Optitrack systems. This section explains in what coordinate system you can expect data to be streamed on ROS with our default system. 

