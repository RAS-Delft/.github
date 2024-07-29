# RAS fleet stack software architecture details
This section elaborates which and how we deploy software across our systems.

Commonly a drone has the following devices upon which we deploy software:
- 1 main microcontroller that interacts with actuator and analog sensors.  (e.g. Arduino, ESP, ...)
- 1 linux based pc with more conputational power, albeit not designed to directly interact with analog systems. (e.g. NUC, Raspberry-Pi ...) 


This image sketches the 2024 vision on structuring drone subsystems
<br>

![ras_system_architecture_workspace drawio](https://github.com/RAS-Delft/ras-documentation-overview/assets/5917472/eadc34ec-44f6-4790-a78f-63b8fbfa0c5d)


## Drone Computational hub
This device is the computer on board of the vessel has the main task of linking ROS with actuators/sensors. Additionally it provides diagnostics and can execute automated computation locally if desired. 
see [this repository](https://github.com/RAS-Delft/ras_ros_low_level_systems)


## Design principles

### Abstraction
Software components should be reused if possible. Elements that are comparable between different vessels/model lines are to come from a single source, to be customized through vessel specific configuration files. Examples of this are: Ros-micro bridge, main microcontroller code. 

### Modularity
Components should be swapped out if reasonable, taking into account complexity, effort, latency and so on. This allows flexibility and usage of external modules. We envision this through:
- Using ROS
- Use standardized interfaces, topics, coordinate systems, topology.

### Deployment
- Control systems in ROS are preferred to be rospackages set up to be included in launchfiles. 
- Containerize elements (commonly with Docker) for flexible deployment
- Version control with Git
- Document

## Common utilities:
- [OvenMediaEngine](https://github.com/AirenSoft/OvenMediaEngine) container for streaming video
- [Traefic](https://hub.docker.com/_/traefik) for remote deployment of services 
