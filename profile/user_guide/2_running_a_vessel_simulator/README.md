# User guide: 2) Running a vessel simulator

**Goal:** Start a ros node that emulates a ship of RAS, that you can interact with.

**Prerequisites:** A pc with ros2 installed.

## Setting up the package
We first need a ros2 workspace where we can put ros packages, if you don't have one yet:
```shell
cd ~            # go to home folder
mkdir ros2_ws   # Make a ros2_ws folder
cd ros2_ws
mkdir src       # Make a src folder where you place/edit all your ros nodes/packages
```

Next we download/copy the [Turtleboat package](https://github.com/RAS-Delft/TurtleBoat/tree/ros2) to this newly made source folder. Download/place manually, clone it with a git manager of your choice. For example with git command line interface:
```shell
cd src
git clone https://github.com/RAS-Delft/TurtleBoat
```
You should now have a 'Turtleboat' folder in your source folder.

Next up we build the ros workspace. 
``` shell
cd ~/ros2_ws    # navigate to the ros2_ws
colcon build    # Build

source ~/ros2_ws/install/setup.bash # Inform ros2 of the existance of the newly built packages
```

## Run the simulator
In the terminal that you sourced the ros2 workspace, we run the turtleboat node. 
```shell
ros2 run turtleboat turtleboatmain --ros-args -r __ns:=/myboat1
```

```
bart@bart-P5820T:~/ros2_ws$ ros2 run turtleboat turtleboatmain --ros-args -r __ns:=/myboat1
[INFO] [1726578464.433489582] [myboat1.turtleboat_sim]:  f_sim=400.0 f_actuator_ref=0.0 f_pub_pos=5.0 f_pub_heading=16.0
[INFO] [1726578466.408039246] [myboat1.turtleboat_sim]:  f_sim=395.0 f_actuator_ref=0.0 f_pub_pos=5.0 f_pub_heading=16.0
[INFO] [1726578468.407858607] [myboat1.turtleboat_sim]:  f_sim=400.0 f_actuator_ref=0.0 f_pub_pos=5.0 f_pub_heading=16.0
[INFO] [1726578470.408133766] [myboat1.turtleboat_sim]:  f_sim=400.0 f_actuator_ref=0.0 f_pub_pos=5.0 f_pub_heading=16.0
[INFO] [1726578472.407452261] [myboat1.turtleboat_sim]:  f_sim=400.0 f_actuator_ref=0.0 f_pub_pos=5.0 f_pub_heading=16.0
```

The simulator should now be running, although the boat has no inputs yet, and will just float around doing nothing. 
Note that most ras ships have fixed namespaces, such as `/RAS_TN_GR/` or `/RAS_TN_RE/` for the green and red titoneri respectively. These are important to distinct topics once we have more ships within the system. 

In another terminal, lets observe existing nodes: (just 1 now)
```shell
ros2 node list
```
```
bart@bart-P5820T:~$ ros2 node list
/myboat1/turtleboat_sim
```

Lets examine available topics:
```shell
ros2 topic list
```

Lets listen to one of the topics with echo:
```shell
ros2 topic info /myboat1/telemetry/heading
```
``` shell
ros2 topic echo /myboat1/telemetry/heading
```

```
bart@bart-P5820T:~$ ros2 topic echo /myboat1/telemetry/heading
data: 5.863059997558594
---
data: 5.863058567047119
---
data: 5.8630571365356445
---
data: 5.86305570602417
---
data: 5.863054275512695
---
data: 5.863052845001221
```

Other topics will have different messagetypes. Some topics show nothing when you echo them, as they are inputs that have nothing connected to them yet. The next step is to give some input to the ship (on the /actuation or /actuation_priority topics) to interact with the virtual vessel. 