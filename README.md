# am_AruCo_tracker
It is a ROS package for the estimation of an AruCo marker's pose by a mvBlueFox-MLC2 series (monocular camera) attached to an aerial manipulator.

## Version information
- Ubuntu: 18.04 
- ROS: melodic
- c: c17
- c++: c++14
- PX4 firmware

## Devices
- Flight controller (FC): PixHawk4
- Camera: mvBlueFox-MLC2 series
    - USB 2.0
    - BGRA, 8bit

## ROS parameters
- ==/publish_rate==:  
- ==/image_conversion_version==


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
