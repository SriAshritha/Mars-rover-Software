IRC 2025

# Mars Rover Software - Arrow Detection & Navigation

## Table of Contents
- [Overview](#overview)
- [System Architecture](#system-architecture)
- [Components](#components)
  - [Arrow Detection](#1-arrow-detection)
  - [Behavior Tree](#2-behavior-tree)
  - [Yaw Angle Processing](#3-yaw-angle-processing)
  - [cmd_vel to Hardware Topics](#4-cmd_vel-to-hardware-topics)
- [Installation & Usage](#installation--usage)

## Overview
This project is part of the Mars Software Team's effort to enhance rover navigation and autonomy. The system integrates multiple components, including:
- **Arrow Detection**: Identifies directional arrows in the environment to aid in navigation.
- **Behavior Tree**: Implements decision-making logic for autonomous behavior.
- **Yaw Angle Processing**: Computes the rover's heading for precise maneuvering.
- **cmd_vel Conversion**: Translates velocity commands into hardware-specific control signals.

## System Architecture
The system follows a modular design, ensuring scalability and adaptability. It consists of the following primary modules:
- **Perception Module**: Arrow detection using computer vision.
- **Navigation Module**: Decision-making with a behavior tree.
- **Control Module**: Converts velocity commands into hardware-specific actions.
- **Localization Module**: Uses IMU and odometry for yaw angle processing.

## Components

### 1. Arrow Detection
- Uses computer vision techniques to identify arrows in the rover’s environment.
- Extracts direction information for navigation.
- Calculated distance from the rover to the arrow using similarity of triangles.

### 2. Behavior Tree
- Implements a structured approach to decision-making.
- Defines different states and transitions for the rover's movement and actions.
- Ensures modular and scalable autonomy logic.

### 3. Yaw Angle Processing
- Calculates the rover’s orientation using the **ZED 2i stereo camera**, which provides depth and positional data.  
- Integrates **IMU data** from the ZED 2i to improve stability and reduce drift in orientation estimation.  
- Enhances rover navigation by providing **real-time yaw angle updates**, allowing precise turns and alignment with target directions.  


### 4. cmd_vel to Hardware Topics
- Converts velocity commands (`cmd_vel`) to motor control signals.
- Ensures smooth translation of software-level navigation commands into physical movement.
- Adapts commands based on rover-specific hardware configurations.

## 🚀 Setup & Build Instructions


# 1️⃣ Open a Terminal and Navigate to the Workspace 
```bash
cd ~/irc_Ws  # Navigate to the workspace
```

# 2️⃣ Build the Workspace
```bash
rm -f build devel # To rebuild again in your system
catkin_make  # Build the ROS workspace
```

# 3️⃣ Source the Environment  
```bash
source devel/setup.bash  # Source the workspace setup
```

# 4️⃣ Run the Arrow Detection Node
```bash
roslaunch arrow_detection arrow.launch
```

# 5️⃣ Launch the Behavior Tree
```bash 
rosrun my_robot_behavior bt_2.py
```

# 6️⃣ Run the Converter
```bash
rosrun my_robot_behavior converter.py
```


## 📬 Reach out via:
🔗 **LinkedIn**: [Mars Research Station, IIITDM](https://www.linkedin.com/company/mars-research-station/posts/?feedView=all) 
