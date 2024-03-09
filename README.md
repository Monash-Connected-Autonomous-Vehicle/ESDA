# ESDA

This repository holds the software stack for the Monash-Connected-Autonomous-Vehicle's IGVC 2024 entry, the ESDA (Electric Self Driving Automobile).

## Setup and launch

### Dependencies

This repository uses various third party drivers to run for example SDKs for the zed camera & pizki multi.
Refer to the respective repositories of these submodules for further information on such dependencies.
In the future, instructions will be written as part of this README

### Building and sourcing

Run the following commands to create, build and source the workspace once all 
other dependencies have been met

```
mkdir esda_ws;
cd esda_ws;
git clone https://github.com/Monash-Connected-Autonomous-Vehicle/ESDA.git;
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

Note that as of now, until a working URDF is implemented, the following static transforms should be run

`ros2 run tf2_ros static_transform_publisher 2 3 1 0 0 0 swiftnav-gnss base_link`

`ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link base_footprint`

## Contributors ✨

Thanks goes to these wonderful people ([emoji key](https://allcontributors.org/docs/en/emoji-key)):

<!-- ALL-CONTRIBUTORS-LIST:START - Do not remove or modify this section -->
<!-- prettier-ignore-start -->
<!-- markdownlint-disable -->
<!-- markdownlint-restore -->
<!-- prettier-ignore-end -->
<!-- ALL-CONTRIBUTORS-LIST:END -->

This project follows the [all-contributors](https://github.com/all-contributors/all-contributors) specification. Contributions of any kind welcome!
