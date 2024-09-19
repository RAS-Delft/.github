# User guide: 5) Starting a control module

**Goal:** Get familliar with running a ros package that controls the ship, rather than doing it manually.

**Prerequisites:** 
- A ship (virtual or physical) running with namespace `RAS_TN_DB`, `RAS_TN_OR` or `RAS_TN_GR`.
- Rviz ship visualisation stack running.
- ras_ros_core_control module package (also used in ship visualisation guide, for the )

## Starting heading controller
Within the ras_core_control_modules repository there are a few folders with tools that can do generic operations that are common across many basic maritime control systems. We want to start a node that controls the heading of the ship. 

If you followed previous tutorials the ras_ros_core_control package should already be in the ros2_ws, built and sourced. 

In a fresh terminal, lets run the node for the ship of our liking:
```shell
ros2 run ras_ros_core_control_modules heading_controller --ros-args -r __ns:=/RAS_TN_GR
```

Now, a simple PID based controller runs that attempts to correct the angle of the ship. However, it outputs a 'desired control effort' and not actuator references. Lets look at the output of the angle controller:

```shell
ros2 topic echo /RAS_TN_GR/reference/controlEffort/torqueZ
```

```
ros2 topic echo /RAS_TN_GR/reference/controlEffort/torqueZ
data: -0.49973729252815247
---
data: -0.49973729252815247
---
data: -0.49973729252815247
---
data: -0.49973729252815247
---
data: -0.49973729252815247
---
data: -0.49973729252815247
---
data: -0.49973729252815247
---
data: -0.49973729252815247
```

## Starting the thrust allocator
Another node can make sure our reference forces are allocated between the actuators. This can be done with many varying approaches. The following example is an approach which controls zz-torque (rotational) and surge (forward/backward) forces, but ignores sway (sideways) resultants. 

```shell
ros2 run ras_ros_core_control_modules TN_control_effort_allocator_nomoto --ros-args -r __ns:=/RAS_TN_GR
```

Although the ship has no active, changing reference angle, it the combined nodes should be attempting to control the ship to a default angle of 0.0rad. 

## Further options:
- Look through other modules in the ras_ros_core_control_modules folder, to get an idea of what tools already have been developed. 
- Play with PID gains within the python file of the heading controller.
- Make a ros node that publishes other reference angles, or a reference force in surge direction so the ship moves forward.