# ESDA

This repository holds the software stack for the Monash-Connected-Autonomous-Vehicle's IGVC 2024 entry, the ESDA (Electric Self Driving Automobile).

## Setup and launch

### Dependencies

This repository uses various third party drivers to run for example SDKs for the zed camera & piksi multi.
Refer to the respective repositories of these submodules for further information on such dependencies.
In the future, instructions will be written as part of this README

### Building and sourcing

Run the following commands to create, build and source the workspace once all 
other dependencies have been met

```
mkdir esda_ws;
cd esda_ws;
git clone -b esda_sim https://github.com/Monash-Connected-Autonomous-Vehicle/ESDA.git;

# import dependencies that cannot be rosdep installed or apt installed
vcs import ESDA < ESDA/esda.repos; 

# zed-ros2-wrapper utilizes a submodule that must be initialised
cd ESDA/zed-ros-2-wrapper;
git submodule update --init;
cd ../../

# install remaining dependencies, build and source
sudo apt-get update; 
rosdep install --from-path ESDA --ignore-src -yr;
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
<table>
  <tbody>
    <tr>
      <td align="center" valign="top" width="14.28%"><a href="https://github.com/AnthonyZhOon"><img src="https://avatars.githubusercontent.com/u/126740410?v=4?s=100" width="100px;" alt="Anthony Oon"/><br /><sub><b>Anthony Oon</b></sub></a><br /><a href="https://github.com/MOnash-Connected-Autonomous-Vehicle/ESDA/commits?author=AnthonyZhOon" title="Code">💻</a> <a href="#ideas-AnthonyZhOon" title="Ideas, Planning, & Feedback">🤔</a> <a href="#projectManagement-AnthonyZhOon" title="Project Management">📆</a> <a href="#question-AnthonyZhOon" title="Answering Questions">💬</a> <a href="#design-AnthonyZhOon" title="Design">🎨</a></td>
      <td align="center" valign="top" width="14.28%"><a href="https://github.com/dylan-gonzalez"><img src="https://avatars.githubusercontent.com/u/45161987?v=4?s=100" width="100px;" alt="Dylan Gonzales"/><br /><sub><b>Dylan Gonzales</b></sub></a><br /><a href="#design-dylan-gonzalez" title="Design">🎨</a> <a href="#ideas-dylan-gonzalez" title="Ideas, Planning, & Feedback">🤔</a> <a href="#mentoring-dylan-gonzalez" title="Mentoring">🧑‍🏫</a> <a href="#question-dylan-gonzalez" title="Answering Questions">💬</a></td>
      <td align="center" valign="top" width="14.28%"><a href="https://github.com/Jokua"><img src="https://avatars.githubusercontent.com/u/47382093?v=4?s=100" width="100px;" alt="William Tioe"/><br /><sub><b>William Tioe</b></sub></a><br /><a href="https://github.com/MOnash-Connected-Autonomous-Vehicle/ESDA/commits?author=Jokua" title="Code">💻</a></td>
      <td align="center" valign="top" width="14.28%"><a href="https://github.com/Jiawei-Liao"><img src="https://avatars.githubusercontent.com/u/105030837?v=4?s=100" width="100px;" alt="Jiawei"/><br /><sub><b>Jiawei</b></sub></a><br /><a href="https://github.com/MOnash-Connected-Autonomous-Vehicle/ESDA/commits?author=Jiawei-Liao" title="Code">💻</a></td>
      <td align="center" valign="top" width="14.28%"><a href="https://github.com/AbBaSaMo"><img src="https://avatars.githubusercontent.com/u/95030427?v=4?s=100" width="100px;" alt="Baaset"/><br /><sub><b>Baaset</b></sub></a><br /><a href="https://github.com/MOnash-Connected-Autonomous-Vehicle/ESDA/commits?author=AbBaSaMo" title="Code">💻</a> <a href="#design-AbBaSaMo" title="Design">🎨</a> <a href="#ideas-AbBaSaMo" title="Ideas, Planning, & Feedback">🤔</a> <a href="https://github.com/MOnash-Connected-Autonomous-Vehicle/ESDA/commits?author=AbBaSaMo" title="Documentation">📖</a></td>
      <td align="center" valign="top" width="14.28%"><a href="https://github.com/Akul-Saigal"><img src="https://avatars.githubusercontent.com/u/108743138?v=4?s=100" width="100px;" alt="Akul Saigal"/><br /><sub><b>Akul Saigal</b></sub></a><br /><a href="#projectManagement-Akul-Saigal" title="Project Management">📆</a></td>
      <td align="center" valign="top" width="14.28%"><a href="https://github.com/Christina1508"><img src="https://avatars.githubusercontent.com/u/108743138?v=4?s=100" width="100px;" alt="Christina Prakash"/><br /><sub><b>Christina Prakash</b></sub></a><br /><a href="#projectManagement-Christina1508" title="Project Management">📆</a></td>
    </tr>
  </tbody>
</table>

<!-- markdownlint-restore -->
<!-- prettier-ignore-end -->

<!-- ALL-CONTRIBUTORS-LIST:END -->

This project follows the [all-contributors](https://github.com/all-contributors/all-contributors) specification. Contributions of any kind welcome!
