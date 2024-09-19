# User guide: 1) Configuring your system

**Goal:** Set up your computer to start interacting with the framework of RAS.

**Prerequisites:** A pc with linux (e.g. Ubuntu) installed that supports ros2 (preferably the latest long-time-supported version). ROS2 versions are dependent on ubuntu version, so take this in mind.


## Installing ROS2
Follow [the official guides](https://docs.ros.org/en/jazzy/Installation.html) to install ros2 on your system. Use the [deb packages](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) to install everything. 

So far, most RAS modules have worked across multiple versions of ROS2. The only trouble I had was with ros2_foxy in collaborating with ros2_iron and beyond. It is likely best to set up systems to work on the latest LTS of ros2/ubuntu. 

Default ros domain ID is 0, which we also use, so it does not need to be configured. 

## Install cyclone RMW implementation:
RMW facilitates communication between nodes for ros on the background, and its modular, meaning we can choose what protocol/provider does this magic for us. Performance differs a bit per supplier/usecase. We use CycloneDDS, and not the default FastDDS

[This guide](https://docs.ros.org/en/jazzy/Installation/DDS-Implementations/Working-with-Eclipse-CycloneDDS.html) explains you how to set up basic cyclonedds. 

best to set RMW_IMPLEMENTATION environment variable in .bashrc (commands that automatically run when you open a terminal) so you won't forget. Navigate to your home folder and open .bashrc in nano editor
```shell
cd ~
sudo nano .bashrc
```
Add the following line:

```shell
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

## Configure CycloneDDS
A few settings need to be configured. 

We set environment variable ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET. This limits the ros2 discovery range for finding other nodes. (subnet = 1 jump over 1 router, and not further.) Also we specify a location (in home folder) of an xml file with cyclone configurations:
```shell
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET # How many network jumps does ros do to find other nodes. 
export CYCLONEDDS_URI="${HOME}/cycloneddsconfig.xml" # To tell cyclone where settings can be found. 

```

Next up we set the cycloneddsconfig.xml with nano:
```shell
cd ~
sudo nano cycloneddsconfig.xml
```

Paste the following, and edit the 'NetworkInterface name' field to the network device that you want to use to connect to the ros2 network of the lab. This example had the ethernet adapter name specified, but this can also be wifi for example. Typing `ifconfig` in a terminal shows you available network adapters.
```xml
<?xml version="1.0" encoding="utf-8"?>
<CycloneDDS>
  <Domain>
    <General>
      <Interfaces>
        <NetworkInterface name="eno1" />
      </Interfaces>
      <AllowMulticast>
        true
      </AllowMulticast>
    </General>
    <Discovery>
      <ParticipantIndex>
        none
      </ParticipantIndex>
    </Discovery>
  </Domain>
</CycloneDDS>
```
Save and exit.