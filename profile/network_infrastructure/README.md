# RAS network details

The general setup is that all devices (ships or on shore computers) are connected to a single network. This can be a virtual private network (VPN) or a local router.


## Data transmission limitations
We have encountered severe latency in some setups. This section attempts to bundle information that helps us configure networks such that we can do our experiments. 

June 2024: Bart Boogmans

After the migration to ROS2 small systems (e.g. joystick control of 3 ships) with very little topics / traffic worked quite well. During the BEP related to swarm algorithms in early 2024 we ran into data transmission limits, where many latency was observed, and unstable connection overall to the ships (both ping and ROS messages). The following approaches were taken:
- Reduce ros message frequencies of various topics
- Enable configuring of communication settings with a rosparameter. Auxillary/debug topics are nice when they can be turned off completely, where other streams can benefit from parameters that limit/regulat publication rates.
- Switching routers from the (supposedly powerful) Netgear Nighthawk (SSID NETGEAR21) to the slightly old TPLink Archer (SSID agv-lab-wifi) did not yield improvements, although it was good to check whether there were issues with the former. 
- Using ethernet cables fixed basically all issues. Wifi throughput is the culprit. Colleagues told me its either volume or total package numbers. Prior traffic monitoring showed data transfer during operation showed magnitudes of 5-50kb/sec outgoing traffic on a drone, which is not a lot so I suspect that its total number of packages that are problematic. We can try eliminating numbers further where possible. 
- To be continued...


## Ros Middleware Implementation
As of 8 august 2024 [BB] switched to cyclonedds. It can be installed with:
``` bash
sudo apt install ros-${ROS_DISTRO}-rmw-cyclonedds-cpp
```
Set RMW implementation and some other suggested settings. normally done in .bashrc. Some settings were based on recommendations from colleagues, referring to others struggling with distributed robotic network setups in [this presentation](https://youtu.be/b_eaXa1m-NA?feature=shared&t=982):
``` bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET # How many network jumps does ros do to find other nodes. 
export CYCLONEDDS_URI="${HOME}/cycloneddsconfig.xml" # To tell cyclone where settings can be found. 
```
Configure cyclonedds with in xml file (I save this in ras user home folder as cycloneddsconfig.xml) with: (replace `wlp2s0` with the name of the network adapter)
``` xml
<?xml version="1.0" encoding="utf-8"?>
<CycloneDDS>
  <Domain>
    <General>
      <Interfaces>
        <NetworkInterface name="wlp2s0" />
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
Also doublecheck (`ifconfig`) if the desired network adapter (here wlp2s0) has the MULTICAST flag, signaling that multicast is enabled. 

## Router Info
A bit of background on the routers in case of future debugging:
- NETGEAR21 Nighthawk X6 R8000 - Commonly used in basement + waterside. Configuration at http://192.168.1.1/
- AGV_Lab-2.4GHz Tplink Archer C7 - Backup router. Configuration at http://tplinkwifi.net/
- NETGEAR32 