# RAS Facility overview
This section describes details on vessels and testing facilities:

## Vessels
There is a wide range of other equipment available that is not listed here or standard mounted on a vessel, such as particular sensors and computers. Ask technicians if you are looking for something in particular, as we might have what you need.

### Delfia
|<p width="100%">    <img width="49%" src="https://github.com/user-attachments/assets/b1b71b13-1b0d-491a-bef0-fc4581ee2c0a">     <img width="41%" src="https://github.com/user-attachments/assets/631eea9c-8869-47c8-93d6-970ca0d5c8d7"> </p> |
| ----- | 
|**Size:** 0.38x0.18m |
|**Amount:** 10 |
|**Thruster setup:** Two continuous rotating azimuths (front & Back). Engines go both forward and backwards. |
|**Equipment:** Proximity sensors (acoustic & IR), although this approach is kind of abandoned. Localisation is commonly done with our ceiling mounted motion tracking system. |
|**Use:** Generally only for inside. Multi vessel operation & cooperation. |

### Tito Neri
|<p width="100%">    <img width="40%" src="https://github.com/user-attachments/assets/172630a1-a4b6-4369-8673-a2aea85236c3">  <div>   <img width="40%" src="https://github.com/user-attachments/assets/d7014aff-efcb-446d-b645-e0ff492ca926"> </p> |
| ----- | 
|**Size:** 0.98x0.33m|
|**Amount:** 7 in various colours|
|**Thruster setup:** Two non continous rotating azimuths at the aft (limits ±135°). Fixed angle bow thruster at the stern. Engines go both forward and backwards.|
|**Equipment:** 9 axis inertial measurement unit (3axis linear & 3axis angular accelleration, 3axis magnetometer), Emlid M2+ RTK GNSS (we have our own station to generate correction messages), optional: cameras|
|**Use:** Inside with optical motion tracking system and outside with ship mounted localization|

### Grey Seabax
|<p width="100%">  <img width="40%" src="https://github.com/user-attachments/assets/d95b3ac9-da7f-4ff8-836e-894e27f35974"> </p> <br> |
| ----- | 
|**Size:** 1.37x0.29m|
|**Amount:** 1|
|**Equipment:** Relatively large deck for various sensors and processing units, such as a Jetson + camera's for neural networks to do optical detection & localisation.|
|**Use:** Inside and outside|

## Testing locations
As indicated below, the inside testing locations have the vertical axis (z) positive in upwards direction when streamed from optitrack. This is not in line with common maritime notations (e.g. North-east-down) but we stream signals on ROS as such, and leave transformations to further modules if desired. 

### Towing Tank
|<p align="center" width="100%">    <img width="60%" src="https://github.com/user-attachments/assets/71019d50-c8f0-4bd0-abf3-5c43b71d030d">  </p> <p align="center" width="100%">   <img width="35%" src="https://github.com/user-attachments/assets/1dfc134f-f932-4c5f-9823-1e243df988c5">     <img width="20%" src="https://github.com/user-attachments/assets/d259bd5a-2339-4b35-bb81-894b24559df2"> </p> |
| ----- | 
|**Size:** ~2.8x80.0m|
| Covered on 40m with Optitrack motion tracking systemthat are shown on the lower pictures, that allow (according to the calibration quality estimator) submilimeter accuracy, streamable between 30 and 300hz. Reflective markers are needed on objects to track them in the intended fashion.|

### Flume Tank
|<p align="center" width="100%">    <img width="70%" src="https://github.com/RAS-Delft/ras-documentation-overview/assets/5917472/1d608763-c2a8-4dd7-90b6-39cc9a6998d1"> </p>|
| ----- | 
|**Size:** Approximately 3.0x5.0m workspace|
|Current settable up to ~0.4m/s. Covered with Optitrack motion tracking system similar as described with the towing tank.|

### Faculty Pond
|<p align="center" width="100%">    <img width="70%" src="https://github.com/user-attachments/assets/4e721890-a07b-4a14-95a0-dd865d5f529c"> </p>|
| ----- | 
|**Location:** [52°00'05.6"N 4°22'19.0"E](https://www.google.com/maps/place/52%C2%B000'05.6%22N+4%C2%B022'19.0%22E/@52.0015637,4.3713033,19z/data=!3m1!4b1!4m4!3m3!8m2!3d52.0015637!4d4.371947?hl=nl&entry=ttu)|
| This needs localisation systems on the ships themselves, where commonly a combination between magnetometer and differential gnss is used. There can be a fair amount of disturbance, ranging from sensor noise, wind to leaves, which create more realistic control challenges. The depth is around 70cm with very soft soil.|

### Tabletop tank
|<p align="center" width="100%">    <img width="70%" src="https://github.com/user-attachments/assets/5f60ef20-a942-4aa7-af11-0863602c8fd2"> </p> |
| ----- | 
|**Size:** 1.28x0.79x0.30m|
| Covered with an arch with optitrack cameras providing localisation over the whole surface|




### RAS Basement tank
|<p align="center" width="100%">    <img width="70%" src="https://github.com/user-attachments/assets/4221c42e-0e8c-43d8-95c6-bf88073f8898"> <img width="30%" src="https://github.com/user-attachments/assets/e13674a4-cc5e-4d80-a1e6-1be8830d137f"> </p> |
| ----- | 
|**Size:** 3.5x4.0x1.8m width, length, depth|
|The purpose of this tank is doing experiments in non-flowing water for surface & underwater robots. It will be equipped with optical motion tracking systems similar to the other tanks (for surface operation), and potentially localization sensors underwater afterwards. |
