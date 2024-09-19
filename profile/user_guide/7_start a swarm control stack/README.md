# User guide: 7) Starting an example control stack: boids swarm control stack

**Goal:** Run the boids swarm control stack which was co-developed with bep students in 2024

**Prerequisites:** 
- Ros2 installed
- [turtleboat](https://github.com/RAS-Delft/TurtleBoat), [urdf_launch](https://github.com/ros/urdf_launch), [ras_urdf_common](https://github.com/RAS-Delft/ras_urdf_common), [ras_ros_core_control](https://github.com/RAS-Delft/ras_ros_core_control_modules) rospackages from previous tutorials in ros2_ws, built and sourced


## Preparation
Add the [rospackage with the swarm control nodes](https://github.com/RAS-Delft/swarm_controller_1)

```shell
cd ~/ros2_ws/src
git clone https://github.com/RAS-Delft/swarm_controller_1
cd ~/ros2_ws
colcon build
source install/setup.bash
```

## Running the stack
```shell
ros2 launch boat_controller swarm.launch.py 
```

This work was made in a bep project and has not matured a lot, although it was functional. Therefore setting up this stack might need further effort to get properly running.