# Parameters and ROS Info

Below is an overview of key topics that are used within the ESDA stack grouped by packages 
that publish them.


## Sensing
### velodyne
|Topic|Type|Objective|Nodes interacting|
------|----|---------|-----------------|
|'/velodyne_points'|'sensor_msgs::msgs::PointCLoud2'|||
|'/velodyne_scan'  |'sensor_msgs::msgs::LaserScan'  |||

### swiftnav-ros2
|Topic|Type|Objective|Nodes interacting|
|------|----|---------|-----------------|
|'/navsatfix'                 |'sensor_msgs::msgs::NavSatFix'                 |||
|'/imu'                       |'sensor_msgs::msgs::imu'                       |||
|'/twistwithcovariancestamped'|'sensor_msgs::msgs::TwistWithCovarianceStamped'|||

[//]: # (confirm which of the many topics Anthony + cam pipeline team are actually using)

### zed-ros2-wrapper
|Topic|Type|Objective|Nodes interacting|
|------|----|---------|-----------------|
|||||

### swiftnav-ros2
|Topic|Type|Objective|Nodes interacting|
|------|----|---------|-----------------|
|||||


### Localization
## robot_localization
|Topic|Type|Objective|Nodes interacting|
|------|----|---------|-----------------|
|||||

### Perception/Obstacle Detection
## ...
|Topic|Type|Objective|Nodes interacting|
|------|----|---------|-----------------|
|||||


### Planning
## navigation2
|Topic|Type|Objective|Nodes interacting|
|------|----|---------|-----------------|
|||||

### Control
## esda_control
|Topic|Type|Objective|Nodes interacting|
|------|----|---------|-----------------|
|||||
