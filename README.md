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
- `/publish_rate`
- `/create_marker`
- `/marker_type`
- `/store_directory`
- `/image_conversion_version`
- `/fx`, `/fy`, `/cx`, `/cy`

## ROS topics
### Subscribing topics

### Publishing topics