# User guide: 8) Starting physical titoneri ships

**Goal:** Replace the earlier used simulators with the physical ships, outside. Set up all the sensor equipment, network and the actuator bridge. 

**Prerequisites:** 
- Ros2 installed on a shore pc
- Joystick teleoperation module (or any other module that used to talk to the simulator) ready on shore pc
- Capabilities to SSH into another device
- Make sure your intended titoneri is charged
- Make sure the Emlid Reach on the tripod is charged

## Preparation
### Connect to one of the ships
Pick one of the ships. Turn it on. The nuc/raspberry pi/onboard pc should turn on automatically. 

Keep track of the power on the batteries in the meantime. Its safe to put the ship on charge when its still in the lab.

With your pc, connect to the NETGEAR21 modem in the laboratory. Ethernet is preferred when the network is stressed, although for simple operations as a single ship joystick teleoperation, wifi suffice. 

Use SSH to connect to one of the ships. All ship pcs use the username 'ras'. IP adresses are dynamically set up, and can be retreived from the modem ( NETGEAR21 router accessible through webbrowser on http://192.168.1.1/ ). 


```shell
ssh ras@192.168.1.12 # Using a specific IP adress of the ship
```
Alternatively you can use the hostname of a ship to access it. 
```shell 
ssh ras@titoneri-darkblue.local
```
```shell 
ssh ras@titoneri-green.local
```
```shell 
ssh ras@titoneri-orange.local
```

If you cannot connect to the ships, and they don't appear in the router's attached devices panel, attach a keyboard/screen to the device and do wifi diagnostics manually. 
### Start the actuator bridge
Run the [ros microcontroller bridge](https://github.com/RAS-Delft/ros_micro_bridge) to let the arduino start executing actuator reference, and return telemetry.
```shell
ros2 run ros_micro_bridge arduino_bridge_ros2 --ros-args --remap __ns:=/${VESSEL_ID}
```
The ship should now be listening to actuator references. It also returns some sensor readings. 

Take a look at topics that are now available from the darkblue ship:
```shell
 ros2 topic list
```

```shell
 ~$ ros2 topic list
/RAS_TN_DB/reference/actuation
/RAS_TN_DB/reference/actuation_prio
/RAS_TN_DB/telemetry/heading
/RAS_TN_DB/telemetry/imu
/RAS_TN_DB/telemetry/micro_serial_stream
/parameter_events
/rosout
```

Feel free to look at stream contents such as /heading, /imu or /micro_serial_stream

```shell
ros2 topic echo /RAS_TN_DB/telemetry/imu
```

Test if the commands of the joystick module are now executed by your ship. 

**Footnotes on this module** <br>
The heading is calculated using the magnetometer readings in horizontal direction, and finding the angle of the vector that is assumed to point north. If you feel that the readings of the heading sensor are incorrect or to noisy, it can help to restart this bridge module (ctrl-c to stop, and re-enter the command) while it is away from any objects that may induce a magnetic field (walls/metal objects), and rotate the vessel 360° with the joystick. 

There is an option (ros parameter that you can set) to correct for magnetic deviation, but we generally don't use it since the errors (commonly ~3°) are often not too disturbing for operating at our scale. 

The `micro_serial_stream` topic is there still for historical reasons. We try to put commonly relevant datastreams in dedicated topics. This stream is an array of numbers that are passed from the arduino to the on board computer, although they are nowadays only used for diagnostics and not control. 

### Start the emlid gnss bridge
Put the Emlid Reach on the tripod outside near your sailing area. Think of a method to remind yourself to not forget it after your tests.

Make sure it is visible by sattelites from as wide angles as possible (e.g. not next to a wall). Turn it on, and don't move it. The device will read its position for 2 minutes, average it as its assumed location, and then start broadcasting a stream of gnss corrections over LoRa.

When you started up the ship, the Emlid M2 gnss module should have also started. It should have the three colored LEDS on top on. Unplug/replug if not. You should be able to ping the M2 (from the ship pc).
```shell
ping 192.168.2.15
```

Open another terminal on your shore pc. SSH into the ship again. Start the [gnss bridge](https://github.com/RAS-Delft/reach_ros_node), which passes on the emlid gnss stream and publishes it on ROS:
```shell
ros2 run reach_ros_node nmea_tcp_driver --ros-args -p host:=192.168.2.15 -p port:=9001 -r __ns:=/${VESSEL_ID} -r tcpfix:=telemetry/gnss/fix -r tcpvel:=telemetry/gnss/tcpvel -r tcptime:=telemetry/gnss/tcptime
```
This should yield a stream of the gnss position on ROS if your ship is outside. '192.168.2.15' is the address of the emlid M2 from the pc and is static. 

Notice how there is also already some remapping of topics, which can be changed when needed. 

If the ship is inside the bridge gives warnings that it has empty messages, which can be ignored. 


**Additional optional checks** <br>
- Both the Emlid Reach beacon and the M2 devices on the ship also connect to the NETGEAR21 router if it is within range. You should be able to see them in the 'connected devices' panel in the overview of the router. Accessing their IP adresses in a browser yields a nice interface with all sorts of details/diagnostics about the GNSS device. 
- Check if the M2 devices on the ships receive correction date from the beacon on shore.
- Check if the M2 devices are able to figure out a good (called a 'fix') solution. 
- Keep an eye out if the solution is still a 'fix' during experiments by using this interface, especially when you operate within proximity of possible obstructions such as near high walls.

## Launch the ship & sail
Start the joystick module. Consider how much the batteries are charged when making a plan. Launch the ship outside and start sailing. 

**Optional: Start any automated control stack**
You can now turn on automated control stacks as well. Be sure to not also accidentally turn on a simulator of a ship, as we now have a physical one that already streams outputs. Stopping/pausing the joystick module in control of a ship releases the priority control to your scripts after a few seconds. If you are done, or if the algorithms fail, you should be able to take over again.


## Shutdown
- Bring your ship inside.
- **Pick up the Emlid tripod from outside.** Turn it off. Charge it if needed. 
- Stop any ros nodes running on the ship (ctrl-c)
- shutdown the pc with: ```sudo halt```
- Charge the battery
