# ESDA Software Guide
This repository holds the software stack for the Monash-Connected-Autonomous-Vehicle's IGVC 2025 entry, the Asterius MKII.
## Basic Setup
Once you boot up the Jetson Orin Nano, you should navigate to the folder called asterius_ws using `cd asterius_ws`.

When you access the folder, the workspace should already have been built, so all that is required is to `source install/setup.bash` before doing anything.

ANY changes made in the `src` folder you will have to rebuild the workspace by running `colcon build --symlink-install`

# ROS2 Control Setup with Arduino Code
After you have run `source install/setup.bash`, you should run the Arduino IDE OR Arduino CLI (Needs updating on how to do this) in order to push the ***ROSArduinoBridge.ino*** code into the arduino board of your choice.

Once you have managed that, run `ros2 launch esda_vehicle_control launch_real.launch.py` in a terminal and it should generate nodes that will receive Twist Commands however we send it, and the node to take note of is */esda_diff_drive_controller/cmd_vel*.

After you have done this, open another terminal, and run `ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/esda_diff_drive_controller/cmd_vel -p stamped:=true`, which launches the in-built teleop node from ROS2 and remaps the */cmd_vel* output from it to 
*/esda_diff_drive_controller/cmd_vel*. Remember to stay in this terminal in order to send it Twist Commands with reference to the layout it displays in the terminal, shown below

![image](https://github.com/user-attachments/assets/1ed7d1d7-9966-4018-ae6f-ef3a38eb28d7)

## ROS2 Control Code Layout
In the event you do need to modify the code, here are some likely scenarios you may face. Take note that EVERYTHING related to ROS2 Control for the Asterius MKII is in the *esda_vehicle_control* folder in the `src` folder of the workspace `asterius_ws`

### USB Port Modification
   
  If the Arduino board isn't being detected despite being plugged in via USB, it is likely that it's the wrong port. Easiest way is to make sure to go for the top right port when facing the port side of the Jetson Orin Nano, however if you're feeling like you want to get your hands dirty, go to the directory `src/ESDA/esda_vehicle_control/description` and find the `ros2_control.xacro` file. You can notice around line 29 that the device is currently set to /dev/ttyACM0. If you need to change that, I would suggest in a fresh terminal or one that isn't busy run `ls /dev/tty*` before AND after you plug the arduino board in to identify which one. Since it's Linux you can narrow down your search to something like *ttyACM* with a number at the end of it.

![image](https://github.com/user-attachments/assets/2b08984d-2f80-446a-b946-4d1fc34eb57d)


NOTE(27/05/2025): In theory, this should be able to send Twist Commands to the Arduino which should translate it to code the motors can understand.

## ZED2i Camera and Velodyne LiDAR Setup
You can refer to the wiki [here](https://github.com/Monash-Connected-Autonomous-Vehicle/ESDA/wiki/Software-Requirements)
