# am_AruCo_tracker
- Ubuntu version: 18.04 
- ROS version: melodic
- c version: c17
- c++ version: c++14

## Devices
### PixHawk4
Firmware version: 1.11.03

### mvBlueFox
<br> Monocular camera </br>
<br> USB 2.0 </br>
<br> Encoder: BGRA, 8bit </br>

## ROS parameters

<br> /publish_rate </br>
<br> /image_conversion_version </br>


### AruCo marker storage
<br> /create_marker </br>
<br> /store_directory </br>

### Camera parameters
<br> /fx </br>
<br> /fy </br>
<br> /cx </br>
<br> /cy </br>
<br> /k_ </br>
<br> /p_ </br>

### Visualization
<br> /vis_flag </br>

## ROS topics
### Subscribing topics
<br> /0/image_raw </br>
<br> /body_to_camera </br>
<br> /mavros/local_position/odom </br>
<br> /mavros/imu/data </br>

### Publishing topics
<br> /aruco_marker/pose </br>
