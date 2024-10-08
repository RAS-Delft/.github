# Researchlab Autonomous Shipping (RAS) - Documentation overview
<p align="center" width="100%">
    <img width="33%" src="https://github.com/user-attachments/assets/74fe3674-ea5b-40d7-bbb1-13b9ec2d2e0f">
</p>

Welcome to the RAS GitHub repository. Here, you'll find essential information about our projects and easy access to related repositories. 

This project is housed at Delft Technical University, faculty Mechanical Engineering. We develop maritime automation systems, and provide experimental opportunities for research. 
See also : [https://rasdelft.nl/](https://rasdelft.nl/) for more elaborate descriptions, partnerships and updates. 

For information on the technicalities on the facilities and software, contact faculty technicians.

## Facilities & Fleet
We have various bodies of water, both inside and outside, where we regulary conduct experiments with any of our 18 available model scale ships. 
The nature of the testing facilities and vessels vary, to cater various research needs. Inside bodies have visual motion tracking systems, whereas outside we rely on ship mounted sensors. 

See more detailed descriptions in the [fleet & facility overview](https://github.com/RAS-Delft/.github/tree/main/profile/facilities)


* Towing Tank, Faculty lake with Tito Neri vessel
<p align="center" width="100%">
    <img width="49%" src="https://github.com/user-attachments/assets/73239bab-b2fd-4a0b-a977-bbd3ca774a23"> 
    <img width="49%" src="https://github.com/user-attachments/assets/518fb4f9-c49b-45b4-9da8-fba9f65dee66"> 
</p>

* Flume Tank with Tito Neri vessel, Basement tank with Delfia vessel
<p align="center" width="100%">
    <img width="49%" src="https://github.com/user-attachments/assets/5eea2430-3a37-4d08-ad30-ce1e90fa7448"> 
    <img width="49%" src="https://github.com/user-attachments/assets/966b21e7-ca25-4b2b-a8bd-db5b3e55afb0"> 
</p>

## User guide
Instructions to start using ras systems for new users

1) [Configuring your system](https://github.com/RAS-Delft/.github/tree/main/profile/user_guide/1_configuring_your_system)
2) [Running a vessel simulator](https://github.com/RAS-Delft/.github/tree/main/profile/user_guide/2_running_a_vessel_simulator)
3) [Visualize the motion of the ship](https://github.com/RAS-Delft/.github/tree/main/profile/user_guide/3_ship%20visualisation)
4) [Take control with a joystick module or your own software](https://github.com/RAS-Delft/.github/tree/main/profile/user_guide/4_joystick_teleoperation)
5) [Use a heading control module](https://github.com/RAS-Delft/.github/tree/main/profile/user_guide/5_start%20a%20(heading)%20control%20module)
6) [Start the benchmark formation control stack](https://github.com/RAS-Delft/.github/tree/main/profile/user_guide/6_start%20a%20formation%20control%20stack)
7) [Start the benchmark Swarm control stack](https://github.com/RAS-Delft/.github/tree/main/profile/user_guide/7_start%20a%20swarm%20control%20stack)
8) [Operate physical ships outside](https://github.com/RAS-Delft/.github/tree/main/profile/user_guide/8_start%20physical%20vessels)

## Default component interfaces
Various signals that are being passed between components (e.g. sensor measurements, control signals) are commonly defined in a certain way within our framework. This is not set in stone, but this page describes our preferred way of defining messages.

## Overview of common modules
Many components are required to make a ship manouver automated, from sensing, various levels of decision-making, communication to actuation. The [control module overview](https://github.com/RAS-Delft/.github/tree/main/profile/module_overviews) describes modules that are used in the common vessel control stacks within RAS. Most components are developed to work with/on the Robotic Operating System. 

This section describes different software components that we have, how they interface with eachother, and where links to their repositories. 
Our vision is to re-use as much modules as possible when doing experimentals, so we can benefit from previous work and mature existing components. 

<p align="center" width="100%">
    <img width="80%" src="https://github.com/user-attachments/assets/f1de20f7-9d4d-4674-aeac-bd8d720fb5df" alt="An example block schematic built from different control modules">
</p>
<p align="center">An example block schematic built from different control modules</p>


## Maintainers guides
For people that are familliar with ROS / RAS systems or that want to work more on the backend of our systems. This is mostly for technical staff / for ourselves, but feel free to snoop around to see how we set things up.
1) [Configuring a ship PC & Microcontroller from scratch](https://github.com/RAS-Delft/.github/tree/main/profile/maintainers_guide/benchmark_vessel_computer_setup#readme)
