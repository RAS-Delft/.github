# Installing ship computer components from scratch
For installing a computer on board of a ship. 

Prerequisites: 
- A board to install onto (e.g. a Nuc, or Raspberry pi)
- An idea of the ROS version you want to deploy
- A ship with a microcontroller, sensors & actuators connected.

There are two ways to deploy software systems on ships here
- Manual; you install all dependencies, you download all required repositories/ros packages, place them, build them. 
- Use Docker. Dockerfiles describe the same steps, but already programmed as if it is a recipe. Its more complex to set up, but easier to deploy/repair on larger systems (e.g. more services per ship/ more ships). The steps for docker are listed as optional. 

Note that if some of the manual setup instructions are unclear/incomplete, consider checking out the docker instructions. The effective steps that the systems are doing are mostly identical. Consider opening a dockerfile and observing what commands are run there, as your solution might be there as well. 

## Installation of OS on on-board computer
Install a recent linux distribution. You can choose a more minimalistic OS, or one with conveniencies such as a desktop GUI. 

For Raspberry Pi's consider using the linux raspberry-pi-imager program from the software center. It's quite convenient. I used Ubuntu-Server-24.04LTS64bit image for RPI4B

Consider compatibilities with this distribution. Docker can give you flexibility in deploying modules, but if you want to run something specific, check if it is supported (e.g. check support for your envisioned ros distibution).

Configure the network to connect to lab routers or VPN.

Configure SSH connection. Make sure you can ssh with a terminal from another PC to the device using `ssh defaultusername@192.168.1.3` (default user is commonly 'ubuntu' or 'pi' depending on the OS you chose) (a helpful [guide](https://phoenixnap.com/kb/ssh-permission-denied-publickey) )

Set username to `ras` with new password. For example with:
```shell
ssh ubuntu@192.168.1.3
```
make `ras` user and add to sudo group
```shell
sudo adduser ras; sudo adduser ras sudo
```
exit and relog as ras
```shell
ssh ras@192.168.1.3
```
```shell
sudo deluser --remove-home ubuntu
```

(optional) Change the device name to something sensible and preferably unique (to allow login by name later)
```shell
sudo nano /etc/hostname
```

(optional) Enable access by hostname (e.g. ssh ras@titoneri-green.local) by 
```shell
sudo nano /etc/systemd/resolved.conf
```
and setting `MulticastDNS=yes`


(optional) Set access keys between device and your pc for easy access. From your host pc that wants access:
```shell
ssh-copy-id ras@192.168.1.3
```
(optional) Install Docker
```shell
snap install docker
sudo groupadd docker # Give docker sudo access. 
sudo usermod -aG docker ${USER} # Add current user to docker group. Might need relog to apply.
```

(optional) install network debug tools
```shell
sudo apt install net-tools
```

## Configure WiFi
In 2024 we set it up as follows. For connecting initially ethernet is a convenient option. 
ssh onto a ship computer and configure using netplan. once logged in edit the network setup file, or create it if it does not exist:
``` shell
sudo nano /etc/netplan/50-cloud-init.yaml
```

Watch indentation. Working settings were as follows. Copy/edit required parts
``` yaml
# This file is generated from information provided by the datasource.  Changes
# to it will not persist across an instance reboot.  To disable cloud-init's
# network configuration capabilities, write a file
# /etc/cloud/cloud.cfg.d/99-disable-network-config.cfg with the following:
# network: {config: disabled}
network:
    ethernets:
        eth0:
            dhcp4: true
            optional: true
    version: 2
    wifis:
        wlan0:
            access-points:
                NETGEAR21:
                    password: fill_in_actual_password_here_without_quotes
            dhcp4: true
            optional: true
```
Then reboot or use ```sudo netplan apply``


## Setup ROS environment & environment variables
All major components are stored in the ras homefolder (`/home/ras`).
Add the configuration file for this drone, where we set the major settings for this device.

Add the following configurations to the .bashrc. Adapt where needed. The RAS_GH_KEY and username are a key for the ras-delft-user, which allow pulling of private repositories in the RAS github. This could be redundant now, as all these components are nowadays public. 


```shell
export VESSEL_ID=RAS_TN_DB # The namespace of your ship. All topics/nodes/services are distinguished from other ships using this.

# These environment variables can help you when pulling private repositories from the ras-github, but otherwise not necessary. 
export RAS_GH_USERNAME="ras-delft-user" 
export RAS_GH_KEY="<FILL IN ras-delft-user pull/write GITHUB KEY>"
```

Install ROS2 and specify connection settings, such as your RMW implementation in line with the rest of the system, as described in the [user guide - system setup](https://github.com/RAS-Delft/.github/tree/main/profile/user_guide/1_configuring_your_system). 

## Configure GNSS module
The GNSS system from Emlid does need some setup on a new device. Connect it and we should be able to identify its name when it pops up in the latest usb events

Check if leds on the receiver are all on. Check if you can already access the network adress to the gnss:
```shell
ping 192.168.2.15
```

Alternatively, `ifconfig` lists the network to the receiver as a semi random name, but similar to `enx7ee4720f03c4`.

If it is not pingable or listed, we try finding the network device:

```shell
sudo dmesg | grep -i usb
```
```
...
[ 2254.571711] usb 1-1.1: Product: ReachM2
[ 2254.571725] usb 1-1.1: Manufacturer: Emlid
[ 2254.584195] cdc_ether 1-1.1:1.0 usb0: register 'cdc_ether' at usb-0000:01:00.0-1.1, CDC Ethernet Device, ee:57:0b:ce:14:b3
[ 2254.592001] cdc_acm 1-1.1:1.2: ttyACM0: USB ACM device
[ 2254.624507] cdc_ether 1-1.1:1.0 enxee570bce14b3: renamed from usb0

```
Our devicename here is `enxee570bce14b3` which we want to use later on.

Alternatively it could be listed with: (here `enxdaa7eb798e93`)
```shell
ip link list
```
```shell
1: lo: <LOOPBACK,UP,LOWER_UP> mtu 65536 qdisc noqueue state UNKNOWN mode DEFAULT group default qlen 1000
    link/loopback 00:00:00:00:00:00 brd 00:00:00:00:00:00
2: enp1s0: <NO-CARRIER,BROADCAST,MULTICAST,UP> mtu 1500 qdisc mq state DOWN mode DEFAULT group default qlen 1000
    link/ether 80:ee:73:f1:5e:e2 brd ff:ff:ff:ff:ff:ff
4: wlp2s0: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 1500 qdisc noqueue state UP mode DORMANT group default qlen 1000
    link/ether 70:66:55:b1:da:0a brd ff:ff:ff:ff:ff:ff
5: enxdaa7eb798e93: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 1500 qdisc fq_codel state UP mode DEFAULT group default qlen 1000
    link/ether da:a7:eb:79:8e:93 brd ff:ff:ff:ff:ff:ff
```

Activate the connection if state=DOWN
```shell
sudo ip link set enxee570bce14b3 up
```

Assign an IP address to the Raspberry Pi's network interface. This IP should be in the same subnet as your USB device, but not conflicting with default receiver adress:192.168.2.15. For example, assign 192.168.2.1
```shell
sudo ifconfig enxee570bce14b3 192.168.2.1 netmask 255.255.255.0 up
```

Finally you should be able to access the emlid (that is by default reachable on `192.168.2.15` . You might need to reboot for settings to apply):
```shell
ping 192.168.2.15
```

## Option (1) - Manual installation & Start
### Installing ros packages
Go to ros2 workspace on the ship pc and copy the required ros packages
```shell
cd ~/ros2_ws/src
git clone https://github.com/RAS-Delft/ros_micro_bridge
git clone https://github.com/RAS-Delft/reach_ros_node
cd ~/ros2_ws
colcon build
source install/setup.bash
```
### Installing dependencies
We need pip
```shell
sudo apt-get install pip
```
Use pip to install pyserial
```shell
pip install pyserial
```

### Flashing the microcontroller
Connect and flash software to the actuator microcontroller. Commonly we used arduino's, although we are not limited to this. The latest version of the control software is found in the [ras_main_micro_driver repository](https://github.com/RAS-Delft/ras_main_micro_driver/tree/main). Flashing can be done with ArduinoIDE or PlatformIO, as you prefer.

If needed, you can find the USB port upon which a device with 'Arduino' in the devicename is connected with:
```shell
echo $(ls -l /dev/serial/by-id/ | grep Arduino | awk '{print $NF}' | awk -F '/' '{print $NF}')
```

If your device does not already have permission for the usb port, edit them with:
```shell
sudo chmod 666 /dev/ttyAMA0 # Fill in the right device port here
```
### Running the microcontroller and gnss bridge
Run the system according to the [user guide: Starting physical vessels](https://github.com/RAS-Delft/.github/tree/main/profile/user_guide/8_start%20physical%20vessels).

## Option (2) Using docker
### Installing the modules
Get the [repository with main dockerfiles](https://github.com/RAS-Delft/ras_ros_low_level_systems) in the home folder ('cd ~')
```shell
git clone https://github.com/RAS-Delft/ras_ros_low_level_systems
```
If you look at the contents of this repository, you can see that its built out of other repositories as well. This system is called having a repository (module) with submodules (other repositories). This structure can allow you to group a set of modules (in our case some ros packages) and update them as one stack. Now we just pulled the main stack with the information about the submodule versions. Lets issue to grab all submodules that our repository specified.

Navigate to folder and update submodules
 ```shell
 cd ras_ros_low_level_systems; git submodule update --init --recursive
 ```

Now the submodule folders should have contents. 

Next up, we run a docker-compose file, that builds all required components as docker images, that serve as modules that we can bring up. This is like preparing a launch file in ros. It builds three things:
- ros-microcontroller bridge
- ros-Emlid bridge
- an image that can push the microcontroller driver 

Build and run the docker containers
```shell
docker compose build
```
Here is an example of contents of a dockerfile (in this case the [ros_arduino_bridge](https://github.com/RAS-Delft/ras_ros_low_level_systems/blob/main/ros_arduino_bridge/Dockerfile)). Note similarities between the steps that are described above in the section on manual installation. 
```dockerfile
# Get argument ROS_DISTRO from the build command or set default value
ARG ROS_DISTRO=iron

# Use the official ROS 2 image
FROM ros:${ROS_DISTRO}-ros-base

ENV VESSEL_ID=default

# Install additional packages
 RUN apt-get update && apt-get install -y \
      pip
#     iputils-ping \
#     ros-iron-xacro

RUN pip install pyserial

# Copy the boid_ros_pkg directory into container ros2_ws/src
COPY ros_micro_bridge/. /ros2_ws/src/ros_micro_bridge

# Build the workspace
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    cd /ros2_ws && \
    colcon build

# Source ros when accessed via bash: (e.g. through `docker exec -it boid_1 /bin/bash`)
#RUN echo ". /entrypoint.sh" >> /root/.bashrc

# Set tasks to be run upon container startup
CMD . /opt/ros/${ROS_DISTRO}/setup.sh && \
    . /ros2_ws/install/setup.sh && \
    ros2 run ros_micro_bridge arduino_bridge_ros2 --ros-args --remap __ns:=/${VESSEL_ID}
```

Note on 24 sept 2024 on the docker stack - Bart Boogmans: Since we switched to cylcone as RMW, this still needs some configurations with the latest dockerfiles. Probably a line that installs cyclonedds in the dockerfiles need to be added, as it now throws an error.
### Flashing microcontroller with Docker image
The microcontroller is responsible for executing desired actuation. Flashing the microcontroller can be done with starting the set-up docker container that 
- autodetects usb port (defaults to device with Arduino in the name)
- Installs flash software (platformIO) and required libraries in temporary container
- flashes the microcontroller C++ software that is included in [this repository](https://github.com/RAS-Delft/ras_ros_low_level_systems), and exits. 

Navigate to the microcontroller driver folder
```shell
cd /home/ras/ras_ros_low_level_systems/microcontroller_driver
```
Build the docker container. Let the installer container do its thing
```shell
docker compose build; docker compose up
```

The microcontroller should now be ready for use.

### Running
We simply call the docker compose file. It will look if the required images exist (they should if they were built in the previous section). If so, it will run the specified components.
```shell
cd ras_ros_low_level_systems
```
``` shell
docker compose up
```
You should see some messages telling you there are two containers running; one for the gnss bridge, and another for the micro_bridge.
