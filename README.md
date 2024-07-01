# ESDA

This repository holds the software stack for the Monash-Connected-Autonomous-Vehicle's IGVC 2024 entry, the ESDA (Electric Self Driving Automobile).

## Setup and launch

### Dependencies
Ensure [ROS2 Humble Hawksbill](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) is installed

This repository also uses various third party drivers to run for example SDKs for the zed camera & piksi multi.
Run the following commands to install these (or refer to their respective ROS2 driver repositories for further instructions):

Install vcstool to help set up the ros2 workspace when cloning and building
```bash
sudo apt-get update
sudo apt install python3-vcstool
```

Run the following in any directory for swiftnav driver deps
```bash
git clone https://github.com/swift-nav/libsbp.git
cd libsbp
git checkout v4.11.0
cd c
git submodule update --init --recursive
mkdir build
cd build
cmake DCMAKE_CXX_STANDARD=17 -DCMAKE_CXX_STANDARD_REQUIRED=ON -DCMAKE_CXX_EXTENSIONS=OFF ../ 
make
sudo make install

sudo apt-get update
sudo apt-get install libserialport-dev
cd ../../../
```

Run the following to install zed driver dependencies
```bash
# install zed_sdk
wget -O zed_sdk https://download.stereolabs.com/zedsdk/4.1/cu121/ubuntu22
chmod +x zed_sdk
./zed_sdk

# base installer for cuda toolkit
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-ubuntu2204.pin
sudo mv cuda-ubuntu2204.pin /etc/apt/preferences.d/cuda-repository-pin-600
wget https://developer.download.nvidia.com/compute/cuda/12.4.1/local_installers/cuda-repo-ubuntu2204-12-4-local_12.4.1-550.54.15-1_amd64.deb
sudo dpkg -i cuda-repo-ubuntu2204-12-4-local_12.4.1-550.54.15-1_amd64.deb
sudo cp /var/cuda-repo-ubuntu2204-12-4-local/cuda-*-keyring.gpg /usr/share/keyrings/
sudo apt-get update
sudo apt-get -y install cuda-toolkit-12-4

# install driver
sudo apt-get install -y nvidia-driver-550-open
sudo apt-get install -y cuda-drivers-550
```

### Building and sourcing

Run the following commands to create, build and source the workspace once all 
other dependencies have been met

```bash
mkdir esda_ws;
cd esda_ws;
git clone https://github.com/Monash-Connected-Autonomous-Vehicle/ESDA.git;

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

Or launch the sim
`ros2 launch esda_sim launch_sim.launch.py`

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
      <td align="center" valign="top" width="20%"><a href="https://github.com/AnthonyZhOon"><img src="https://avatars.githubusercontent.com/u/126740410?v=4?s=100" width="100px;" alt="Anthony Oon"/><br /><sub><b>Anthony Oon</b></sub></a><br /><a href="https://github.com/MOnash-Connected-Autonomous-Vehicle/ESDA/commits?author=AnthonyZhOon" title="Code">💻</a> <a href="#ideas-AnthonyZhOon" title="Ideas, Planning, & Feedback">🤔</a> <a href="#projectManagement-AnthonyZhOon" title="Project Management">📆</a> <a href="#question-AnthonyZhOon" title="Answering Questions">💬</a> <a href="#design-AnthonyZhOon" title="Design">🎨</a></td>
      <td align="center" valign="top" width="20%"><a href="https://github.com/dylan-gonzalez"><img src="https://avatars.githubusercontent.com/u/45161987?v=4?s=100" width="100px;" alt="Dylan Gonzales"/><br /><sub><b>Dylan Gonzales</b></sub></a><br /><a href="#design-dylan-gonzalez" title="Design">🎨</a> <a href="#ideas-dylan-gonzalez" title="Ideas, Planning, & Feedback">🤔</a> <a href="#mentoring-dylan-gonzalez" title="Mentoring">🧑‍🏫</a> <a href="#question-dylan-gonzalez" title="Answering Questions">💬</a></td>
      <td align="center" valign="top" width="20%"><a href="https://github.com/Jokua"><img src="https://avatars.githubusercontent.com/u/47382093?v=4?s=100" width="100px;" alt="William Tioe"/><br /><sub><b>William Tioe</b></sub></a><br /><a href="https://github.com/MOnash-Connected-Autonomous-Vehicle/ESDA/commits?author=Jokua" title="Code">💻</a></td>
      <td align="center" valign="top" width="20%"><a href="https://github.com/Jiawei-Liao"><img src="https://avatars.githubusercontent.com/u/105030837?v=4?s=100" width="100px;" alt="Jiawei"/><br /><sub><b>Jiawei</b></sub></a><br /><a href="https://github.com/MOnash-Connected-Autonomous-Vehicle/ESDA/commits?author=Jiawei-Liao" title="Code">💻</a></td>
      <td align="center" valign="top" width="20%"><a href="https://github.com/AbBaSaMo"><img src="https://avatars.githubusercontent.com/u/95030427?v=4?s=100" width="100px;" alt="Baaset"/><br /><sub><b>Baaset</b></sub></a><br /><a href="https://github.com/MOnash-Connected-Autonomous-Vehicle/ESDA/commits?author=AbBaSaMo" title="Code">💻</a> <a href="#design-AbBaSaMo" title="Design">🎨</a> <a href="#ideas-AbBaSaMo" title="Ideas, Planning, & Feedback">🤔</a> <a href="https://github.com/MOnash-Connected-Autonomous-Vehicle/ESDA/commits?author=AbBaSaMo" title="Documentation">📖</a></td>
    </tr>
    <tr>
      <td align="center" valign="top" width="20%"><a href="https://github.com/Akul-Saigal"><img src="https://avatars.githubusercontent.com/u/108743138?v=4?s=100" width="100px;" alt="Akul Saigal"/><br /><sub><b>Akul Saigal</b></sub></a><br /><a href="#projectManagement-Akul-Saigal" title="Project Management">📆</a></td>
      <td align="center" valign="top" width="20%"><a href="https://github.com/Christina1508"><img src="https://avatars.githubusercontent.com/u/101304772?v=4?s=100" width="100px;" alt="Christina Prakash"/><br /><sub><b>Christina Prakash</b></sub></a><br /><a href="#projectManagement-Christina1508" title="Project Management">📆</a></td>
      <td align="center" valign="top" width="20%"><a href="https://github.com/jang0029"><img src="https://avatars.githubusercontent.com/u/141693960?v=4?s=100" width="100px;" alt="Jason Angus"/><br /><sub><b>Jason Angus</b></sub></a><br /><a href="https://github.com/MOnash-Connected-Autonomous-Vehicle/ESDA/commits?author=jang0029" title="Code">💻</a> <a href="#data-jang0029" title="Data">🔣</a></td>
    </tr>
  </tbody>
</table>

<!-- markdownlint-restore -->
<!-- prettier-ignore-end -->

<!-- ALL-CONTRIBUTORS-LIST:END -->

This project follows the [all-contributors](https://github.com/all-contributors/all-contributors) specification. Contributions of any kind welcome!
