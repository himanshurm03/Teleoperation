# Teleoperation
Teleoperation framework integrating a Geomagic Touch haptic device with a UR3e robot using ROS Noetic. Supports real-time pose control, haptic force feedback, delay measurement.

## 🚀 Features
- ✅ Real-time pose control of UR3e robot via Geomagic Touch Haptic Device  
- ✅ Force feedback is delivered through the ATI Gamma force/torque sensor mounted on the UR3e robot.
- ✅ Delay measurement for both **pose** and **force feedback**  
- ✅ Logging and CSV export for delay analysis   
- ✅ Camera integration for live video streaming

## ⚙️ Requirements
- **ROS Noetic**  
- **Python 3**  
- UR3e robot with ROS driver  
- Geomagic Touch with OpenHaptics SDK  
- Ubuntu 20.04 (Tested)

# Teleoperation Setup (Geomagic Touch – UR3e)

This repository contains the required codes and instructions to set up and run the teleoperation system between the **Geomagic Touch haptic device** (master side) and the **UR3e robot** (slave side).

---

## Steps to Run the Setup

### 1.Set the Joint Positions First then Launch Force torque Sensor

Ensure the UR3e robot joints are initialized with the following positions (in degrees):

Base: 0
Shoulder: -90
Elbow: -90
Wrist1: -90
Wrist2: 90
Wrist3: 0

### 2. Launch the Force Torque Sensor
roslaunch rpi_ati_net_ft ati_net_ft_driver.launch

### 3.Run this:
rosrun ur_robot_driver error_for_force_revised.py
This will whether the Force Torque sensor is calibrated or not.

### 4.This will give Euler angles in radians (which will be used for the calibration of the haptic device tip with ur3e end effector):
rosrun ur_robot_driver error_for_pose.py

### 5.To run the setup: 
Run this on the master side :
roslaunch aiims_teleop version1.launch
Contents of the version1.launch(master side)
1. version0_2_1_1.py
2.final_haptic_camera_node_2.py

Run this on the slave side :
roslaunch ur_robot_driver version1.launch
Contents of version1_1.launch (slave side)
1.version0_pose1.py
2.version0_force_1.py
3.camera_nod2_test1.py

Then Press : play in Control Section in Run window in Teach Pendant.
### The whole setup will work perfectly.
