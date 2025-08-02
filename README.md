This project builds on the official ROBOTIS OpenManipulator repositories and packages. We acknowledge and thank ROBOTIS for their open-source contributions, which serve as the foundation for this work:

open_manipulator_perceptions (https://github.com/ROBOTIS-GIT/open_manipulator_perceptions)
→ Provides AR marker detection and perception integration in ROS.

open_manipulator (https://github.com/ROBOTIS-GIT/open_manipulator)
→ Includes the controller, hardware interface, and motion logic for OpenManipulator-X.

These repositories are licensed under open-source terms by ROBOTIS. This project was developed for academic and educational purposes by extending their functionality.


# Final-Year Project: Autonomous Pick-and-Place System Using OpenManipulator-X

This repository contains the source code, libraries, configuration files, and documentation for the development of a real-time autonomous robotic sorting system using the OpenManipulator-X robotic arm and ROS Noetic. The project was implemented for a Final Year Project in the Mechatronics Engineering program at Universiti Teknologi Malaysia (UTM).

## 📌 Project Overview

The system is capable of detecting objects tagged with AR markers using a USB camera and the `ar_track_alvar` ROS package. The OpenManipulator-X robotic arm then picks and places these objects into designated locations based on the marker ID. This is achieved using real-time image processing, coordinate transformation, and inverse kinematics implemented within the ROS framework.

[![Watch the video](https://img.youtube.com/vi/AgxemAD3xE8/hqdefault.jpg)](https://www.youtube.com/watch?v=AgxemAD3xE8)


---

## 🗂️ Repository Structure

```
Final-Year-Project/
├── ar_track_alvar/                  # AR marker detection package (Alvar-based)
├── DynamixelSDK/                    # SDK for Dynamixel motor communication
├── Final_Year_Project_Report/       # Documentation and project report
├── image_pipeline/                  # ROS image pipeline tools (calibration, view, processing)
├── open_manipulator/                # Core packages for OpenManipulator-X robot
├── open_manipulator_controls/       # Additional controller and hardware interface packages
├── open_manipulator_dependencies/   # External plugins and dependencies (e.g., Gazebo)
├── open_manipulator_ikfast_plugin/  # Plugin for MoveIt! IK solver (IKFast)
├── open_manipulator_moveit_python/  # Python interface for MoveIt! motion planning
├── open_manipulator_msgs/           # Custom messages and services used across packages
├── open_manipulator_perceptions/    # Integration of perception modules (camera, AR markers)
├── open_manipulator_simulations/    # Gazebo simulation environment
└── README.md                        # Project documentation
```

---

## 🚀 How to Launch the System

Open the following terminals in sequence to launch the full pipeline:

**Terminal 1: Start the ROS Master**
```bash
roscore
```

**Terminal 2: Launch OpenManipulator Controller**
```bash
roslaunch open_manipulator_controller open_manipulator_controller.launch usb_port:=/dev/ttyACM0 baud_rate:=1000000
```

**Terminal 3: Start AR Marker Detection**
```bash
roslaunch open_manipulator_ar_markers ar_pose.launch
```

**Terminal 4: Start the Pick-and-Place Node**
```bash
roslaunch open_manipulator_pick_and_place open_manipulator_pick_and_place.launch
```

> Ensure that the camera is positioned overhead, the markers are printed and visible, and the workspace is well-lit and planar.

---

## 📑 Final Year Report

The full documentation for this project is located in:

```
Final_Year_Project_Report/fyp.md
```

This includes detailed explanations of system architecture, methodology, camera calibration, kinematics, testing procedures, and result analysis.

---

## ⚙️ Dependencies

- ROS Noetic (Ubuntu 20.04)
- OpenCV
- Dynamixel SDK
- `ar_track_alvar` package
- OpenManipulator-X libraries (by ROBOTIS)

---

## 📘 License

This project uses open-source components. Refer to the license files in each package for respective license information.

---

## 👨‍🔧 Developed By

Ahmed Abdulgader Salim Assagaf  
Universiti Teknologi Malaysia (UTM)  
Final Year Project — Mechatronics Engineering
