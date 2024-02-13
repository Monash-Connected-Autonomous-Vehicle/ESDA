# ESDA

This repository holds the software stack for the Monash-Connected-Autonomous-Vehicle's IGVC 2024 entry, the ESDA (Electric Self Driving Automobile).

# Setup and launch

### Dependencies

This repository uses various third party drivers to run for example SDKs for the zed camera & pizki multi.
Refer to the respective repositories of these submodules for further information on such dependencies.
In the future, instructions will be written as part of this README

### Building and sourcing

Assuming this repo has been cloned into the root of a ROS2 workspace and that the root of the workspace
is the current working directory, run the following commands to build and source the workspace once all 
other dependencies have been met

```
cd ESDA;
git submodule update --init --recursive;
cd ../;
rosdep install --from-path ESDA --ignore-src -yr
colcon build;
source install/setup.bash
```

### Launching

The following can be used to launch the ESDA (once implemented)

`ros2 launch esda_launch esda_launch.py`

# Contributors

TODO
