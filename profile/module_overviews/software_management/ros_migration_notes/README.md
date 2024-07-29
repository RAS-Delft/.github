# Documentation on migration from ROS1 to ROS2 & Git restructuring
August 16 2023 - Bart Boogmans

## Introduction
There is a need for migrating the RAS software modules from ros1 to ros2 as ros1 will no longer be supported soon. Besides this, also a new structure is envisioned in the software distribution over repositories. 
This file documents thoughts and progress on these developments. 

The goal is to create a working vessel control system, with all features as implemented now, but then operating in ros2 and logically and intuitively saved across repositories. 

## Git repository restructuring
There are now various fundamental software components spread over repositories. This is usually depicted in some theme, but through time some fundamental works have been created in repositories that now have a broader use than its origonal purpose and name. This causes confusion, and makes software hard to find.
Restructuring this allows formation of an intuitive structure where components are in logical places, findable by other users as well besides the creator. 
The new structure is envisioned as follows:
1) Base ros node components sorted (as much as possible) per theme, such as
- control effort allocators
- motion controllers
- State estimators / sensor-filters
2) Project specific software
  - Formation control 1 demo
  - waypoint following app
  - Swarm control 1 demo

Besides making the software more logically organised, it would also be nice if the restructuring would support open source sharing of the developed control components. 
Ideally, the base components can be shared fully open source, and some of the specific software can be shared, decided per project.

## ROS2 migration
Migration is envisioned in the following steps
  - Port a simulator (nausbot - to use as development tool in this process)
  - Port visualization tools ( to evaluate performance during migration effectiveness)
  - Port Human control interfaces (Matlab joystick app?)
  - Port fundamental ros nodes
  - Port project specific software repositories (that have use in the near future)

We should also check if ros2 communication performs adequately for our purposes (as Janusz mentioned he found limitations there that need investigation)

Components to transfer in 2.0 are as follows.

|    | Module name                          | Migrated | Assigned to | Initial location                                                                                                               |
|----|--------------------------------------|----------|-------------|--------------------------------------------------------------------------------------------------------------------------------|
| 1  | Nausbot simulator                    | Yes      | Bart        | https://github.com/bartboogmans/Nausbot/tree/ros2                                                                              |
| 2  | Ras web diagnostics                  | No       | Casper      | https://github.com/RAS-Delft/web-diagnostics                                                                                   |
| 3  | Joystick bridge                      | Yes      | Bart        | https://github.com/RAS-Delft/RAS_General/blob/main/matlab/GUI%20%26%20control%20subsystems/Joystick_app_directJoyControl.mlapp |
| 4  | Heading controller                   | No       | Bart        | https://github.com/RAS-Delft/USV_surge_velocity_controller/tree/main/src/usv_surge_vel_contr/scripts                           |
| 5  | Emlid reach bridge                   | ?        | Casper      | https://github.com/RAS-Delft/reach_bridge                                                                              |
| 6  | ROS_arduino_bridge                   | Yes      | Casper      | https://github.com/RAS-Delft/RAS_TitoNeri/tree/main/ras_low_level_bridge                                                       |
| 7  | Velocity differentiator              | No       | Bart        | https://github.com/RAS-Delft/USV_surge_velocity_controller/tree/main                                                           |
| 8  | 2dof control effort allocator nomoto | No       | Bart        | https://github.com/RAS-Delft/USV_surge_velocity_controller/tree/main                                                           |
| 9  | geopos/heading differentiator        | No       | Bart        | https://github.com/RAS-Delft/USV_geopos_heading_differentiator                                                                 |
| 10 | Formation control demo               | No       | Bart        | https://github.com/RAS-Delft/USV_formation_control_1/tree/main/src/usv_formation_control_1/scripts                             |

## Versioning
We refer to the RAS 1.0 stack to the functionality that our system has today working on ros1, having various automated control modules and a formation control demo.
We refer to the RAS 2.0 stack to the first functional system with all above described features working over ros2 in reorganised repositories.

## Documentation
All changes should be incorporated on the RAS software overview. (e.g. https://github.com/RAS-Delft/comFrame or some other general location or format (such as caspers mdbook))

## Rollout and evaluation
Before the 2.0 version is fully operational, we can rely on 1.0 to facilitate demonstration and research requests. 
For our users, no conceptual usage will differ, besides the ros version that they should be instructed to work with. 

Close off should reflect lessons learnt and future steps

## Further steps
These are some ideas to follow up afterwards:
- port [mechatronics course](https://github.com/RAS-Delft/MT44000) to ros2?
- Port [towing+ flume tank bridge](https://github.com/RAS-Delft/ros_optitrack_bridge) to ros2
- Make DP+nomoto controlled combined auto switching framework?
- from above: involve standardized pathplanning
- from above: Involve (dynamic) collisionavoidance
- Boids fleet swarming
