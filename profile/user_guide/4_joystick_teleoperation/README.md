# User guide: 4) Joystick teleoperation

**Goal:** Take control of the ship with a joystick

**Prerequisites:** A ship simulator running in the background. Preferably a visualizer running to see what we are doing. 


## Preparation
Clone the [ras_joystick_control](https://github.com/RAS-Delft/ras_joystick_control) rospackage in your rosworkspace
``` shell
cd ~/ros2_ws/src
git clone https://github.com/RAS-Delft/ras_joystick_control
```

Build and source
``` shell
cd ~/ros2_ws
colcon build
source install/setup.bash
```

## Run the joystick control module
Plug in the Logitech Extreme 3D PRO joystick in a usb port. 

```
ros2 run joystick_control_ras joystickgui
```

Type the namespace/vessel_id of the vessel that you want to control in the field. Press start. 

You should be controlling the vessel now. 
<p align="center" width="100%">
    <img width="33%" src="https://github.com/RAS-Delft/ras_joystick_control/assets/5917472/2161ca47-cb0b-45f9-b8db-09980b44ceff">
</p>


## Evaluating actuation messages
Take a look at the message thats being sent to the ship. Notice how it is structured. 
```shell
ros2 topic echo /RAS_TN_DB/reference/actuation_prio
```
```
---
header:
  stamp:
    sec: 1726581788
    nanosec: 403625327
  frame_id: ''
name:
- SB_aft_thruster_propeller
- PS_aft_thruster_propeller
- BOW_thruster_propeller
- SB_aft_thruster_joint
- PS_aft_thruster_joint
position:
- 0.0
- 0.0
- 0.0
- -0.3665191429188092
- -0.3665191429188092
velocity:
- 1568.0
- 1568.0
- 30.0
- 0.0
- 0.0
effort: []
---
```


This is a standard jointstate message type from ros, with names of joints and position,velocity and effort fields. These are the reference states for the ships actuators which the simulation and the real ship execute. 

## About actuation priority
You might have noticed the suffix '_prio' at the end of the topic name. This refers to the fact that the ships are designed to use this topic with priority above the normal actuation topic. Handy, in case of emergency. Future automated control inputs should thus be sent to the regular actuation topic, such that we can take over if needed. Ships that have been controlled through a priority channel will fall back to the regular actuation topic if the priority topic is inactive for a few seconds. 