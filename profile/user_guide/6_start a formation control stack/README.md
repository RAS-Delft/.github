# User guide: 6) Starting an example control stack: formation keeping

**Goal:** Run the control stack which was used in the 2023 AVATAR deliverable

**Prerequisites:** 
- Ros2 installed
- [turtleboat](https://github.com/RAS-Delft/TurtleBoat), [urdf_launch](https://github.com/ros/urdf_launch), [ras_urdf_common](https://github.com/RAS-Delft/ras_urdf_common), [ras_ros_core_control](https://github.com/RAS-Delft/ras_ros_core_control_modules) rospackages from previous tutorials in ros2_ws, built and sourced


## Preparation
We need to also add the [formation control package](https://github.com/RAS-Delft/USV_formation_control_1) from the ras github:
```shell
cd ~/ros2_ws/src
git clone https://github.com/RAS-Delft/USV_formation_control_1
cd ~/ros2_ws
colcon build
source install/setup.bash
```

## Running 
Simply start the launchfile in that package, which should bring up all major components.
```shell
ros2 launch usv_formation_control_1 formation1.launch.py
```

You should have 3 vessels sailing around a virtual pond (geographical locations are as if they are in front of the mechanical engineering faculty, in the pond.)

This launch file got a bit larger, with many components now started with a single command. This is common practice with ros2 to bring up/kill complex systems. 


## Suggestions
- Look at various nodes, topics and their function
- use 'rqt_graph' in a new terminal to visualize the flow of control signals between nodes.