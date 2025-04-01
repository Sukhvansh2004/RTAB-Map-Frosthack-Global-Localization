
# **Autonomous Initial Localization with RTAB-Map**  

This repository implements an **autonomous initial localization approach** using **visual SLAM** with **RTAB-Map**. Unlike traditional approaches requiring **teleoperation before localization**, this stack autonomously localizes the robot by building a detailed **3D map** of its environment. The implementation is designed to work in both **Gazebo simulation environments** and **physical robot hardware**.  

## **Table of Contents**  

- [Overview](#overview)  
- [RTAB-Map Technology](#rtab-map-technology)  
- [System Requirements](#system-requirements)  
- [Installation](#installation)  
- [Usage](#usage)  
  - [Mapping Mode](#mapping-mode)  
  - [Localization Mode](#localization-mode)  
- [System Architecture](#system-architecture)  
- [Performance Considerations](#performance-considerations)  
- [Troubleshooting](#troubleshooting)  

---

## **Overview**  

This project leverages **RTAB-Map (Real-Time Appearance-Based Mapping)** to perform **visual SLAM** using an **RGB-D camera**. Our implementation is enhanced by integrating **LiDAR sensors** to improve mapping accuracy and reliability.  

This **multi-sensor approach** ensures robust performance in environments with:  
- Varying lighting conditions  
- Limited visual features  
- Large-scale, complex environments  

The system autonomously:  
1. **Creates a detailed 3D map** of the environment.  
2. **Uses that map for accurate localization** without manual teleoperation.  

---

## **RTAB-Map Technology**  

RTAB-Map is a **graph-based SLAM framework** designed for robust **long-term and large-scale mapping** in dynamic environments.  

### **Core Features**  

- **Graph-Based SLAM Architecture**:  
  - Uses a **pose-graph representation** where **nodes represent keyframes** and **edges represent transformations** between them.  
- **Loop Closure Detection**:  
  - Detects previously visited locations using a **Bag-of-Words (BoW) approach** with **SURF/SIFT** features, correcting **pose drift**.  
- **Multi-Sensor Fusion**:  
  - Supports **RGB-D cameras, LiDAR, stereo cameras, and IMU sensors** for enhanced accuracy.  
- **Memory-Efficient Processing**:  
  - Uses **intelligent memory management** to handle **large-scale environments** by keeping only relevant map sections in active memory.  

### **LiDAR Integration**  

Even though RTAB-Map is primarily **visual SLAM**, our implementation integrates **LiDAR sensors** to:  
✅ **Improve depth accuracy** in low-light conditions.  
✅ **Enhance mapping in feature-poor environments** (e.g., plain walls).  
✅ **Provide high-resolution point clouds** for precise localization.  
✅ **Ensure robustness in dynamic environments**.  

The fusion of **RGB-D and LiDAR data** is performed using **RTAB-Map’s multi-sensor registration**, creating a **more complete** and **accurate environmental map** than either sensor alone.  

---

## **System Requirements**  

### **Software Prerequisites**  
- **ROS 2 Humble Hawksbill** (Full desktop installation recommended)  
- **Navigation2 (Nav2) Stack** (for autonomous navigation)  
- **Gazebo Classic** (Version 11.10.0 or newer for simulation)  
- **RViz2** (for visualization and monitoring)  
- **RTAB-Map ROS Package** (Version 0.21.0 or newer)  

---

## **Installation**  

### **1. Set Up Your ROS 2 Workspace**  

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### **2. Clone the Repository**  

```bash
git clone https://github.com/Sukhvansh2004/RTAB-Map-Frosthack-Global-Localization.git
```

### **3. Install RTAB-Map ROS Package**  

```bash
cd ~/ros2_ws/src
git clone https://github.com/introlab/rtabmap.git
git clone --branch ros2 https://github.com/introlab/rtabmap_ros.git
cd ..
```

For detailed installation instructions, visit the official RTAB-Map ROS repository:  
🔗 **[RTAB-Map ROS GitHub](https://github.com/introlab/rtabmap_ros/tree/ros2#rtabmap_ros)**  

### **4. Install Additional Dependencies**  

```bash
cd ~/ros2_ws
sudo apt update
sudo apt install -y ros-humble-nav2-bringup ros-humble-slam-toolbox \
                    ros-humble-gazebo-ros-pkgs ros-humble-robot-localization \
                    ros-humble-laser-filters
rosdep install --from-paths src --ignore-src -r -y
```

### **5. Build the Workspace**  

```bash
cd ~/ros2_ws
colcon build
source ~/ros2_ws/install/setup.bash
```

---

## **Usage**  

### **Mapping Mode**  

This mode allows the robot to **autonomously explore** and **build a map**.  

#### **1. Launch Mapping**  

```bash
ros2 launch tortoise_gazebo rtab_tortoise.launch.py
```

#### **2. Manually Drive the Robot for Proper Mapping**  

Open a **new terminal** and run:  

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Use the **keyboard controls** to move the robot around the world and ensure a complete map.  

#### **3. Check for Loop Closure (Important!)**  

- In **RTAB-Map**, go to the **View** tab and select **Statistics**.  
- It is **essential** to have **at least one loop closure** for localization to work.  
- If no loop closure is detected, **revisit previous locations** to improve map consistency.  

---

### **Localization Mode**  

Once the mapping is complete, use the saved map for localization.  

#### **Launch Localization**  

```bash
ros2 launch tortoise_gazebo rtab_tortoise.launch.py localization:=True
```

---

## **System Architecture**  

The system integrates multiple components, as shown in the diagram:  

```
┌────────────────┐    ┌─────────────────┐    ┌───────────────┐
│  RGB-D Camera  │───▶│ Feature Extractor│───▶│ Loop Closure  │
└────────────────┘    └─────────────────┘    │   Detection   │
       ▲                                      └───────┬───────┘
       │                                              │
┌──────┴───────┐    ┌─────────────────┐    ┌─────────▼───────┐
│ Wheel Odometry│◀───│ Pose Estimation │◀───│ Graph Optimizer │
└──────────────┘    └─────────────────┘    └─────────────────┘
       ▲                     ▲                      │
       │                     │                      │
┌──────┴───────┐    ┌───────┴─────────┐   ┌────────▼────────┐
│  LiDAR Scan  │───▶│ Point Cloud Proc│───▶│  Map Publisher  │
└──────────────┘    └─────────────────┘   └─────────────────┘
                                                   │
                     ┌───────────────────────────────────────┐
                     │              Nav2                     │
                     └───────────────────────────────────────┘
```

---

## **Troubleshooting**  

### **Common Issues**  

#### **Poor Localization Accuracy**  
- Ensure the environment has **sufficient visual features**.  
- Verify **camera and LiDAR calibration**.  
- Check **loop closure statistics**.  

#### **High CPU/Memory Usage**  
- Reduce **map resolution**.  
- Decrease **RGB-D image processing rate**.  
- Enable **point cloud downsampling**.  

#### **Build Issues**  
- If you face issues with `colcon build --symlink-install`, try:  

  ```bash
  colcon build
  ```

#### **Simulation Launch Failures**  
- If **Gazebo or RViz fails to launch**, terminate stuck processes:  

  ```bash
  pkill -9 -f gz
  pkill -9 -f ros2
  ```

