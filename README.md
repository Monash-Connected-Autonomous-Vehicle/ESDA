# ESDA Software Guide
This repository holds the software stack for the Monash-Connected-Autonomous-Vehicle's IGVC 2025 entry, the Asterius MKII.
## Basic Setup
Once you boot up the Jetson Orin Nano, you should navigate to the folder called asterius_ws using `cd asterius_ws`.

When you access the folder, the workspace should already have been built, so all that is required is to `source install/setup.bash` before doing anything.

ANY changes made in the `src` folder you will have to rebuild the workspace by running `colcon build --symlink-install`

## ROS2 Control Setup with Arduino Code
After you have run `source install/setup.bash`, you should run the Arduino IDE OR Arduino CLI (Needs updating on how to do this) in order to push the ***ROSArduinoBridge.ino*** code into the arduino board of your choice.

Once you have managed that, run `ros2 launch esda_vehicle_control launch_real.launch.py` in a terminal and it should generate nodes that will receive Twist Commands however we send it, and the node to take note of is */esda_diff_drive_controller/cmd_vel*.

After you have done this, open another terminal, and run `ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/esda_diff_drive_controller/cmd_vel -p stamped:=true`, which launches the in-built teleop node from ROS2 and remaps the */cmd_vel* output from it to 
*/esda_diff_drive_controller/cmd_vel*. Remember to stay in this terminal in order to send it Twist Commands with reference to the layout it displays in the terminal, shown below

![image](https://github.com/user-attachments/assets/1ed7d1d7-9966-4018-ae6f-ef3a38eb28d7)

NOTE(27/05/2025): In theory, this should be able to send Twist Commands to the Arduino which should translate it to code the motors can understand.

## ZED2i Camera and Velodyne LiDAR Setup
You can refer to the wiki [here](https://github.com/Monash-Connected-Autonomous-Vehicle/ESDA/wiki/Software-Requirements)
