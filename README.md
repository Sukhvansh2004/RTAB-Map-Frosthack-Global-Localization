# Autonomous Initial Localization with RTAB-Map

This repository implements an autonomous initial localization approach using visual SLAM with RTAB-Map. Unlike traditional approaches requiring teleoperation before localization, this stack autonomously localizes the robot by building a detailed 3D map of its environment. The implementation is designed to work in both Gazebo simulation environments and on physical robot hardware.

## Table of Contents

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

## Overview

This project leverages RTAB-Map (Real-Time Appearance-Based Mapping) to perform visual SLAM using an RGB-D camera. While RTAB-Map primarily uses visual data, our implementation has been enhanced by integrating LiDAR sensors to improve mapping accuracy and reliability. This multi-sensor approach provides robust performance in environments with varying lighting conditions or limited visual features. The system autonomously creates a detailed 3D map of the environment and uses that map for accurate localization without manual teleoperation.

## RTAB-Map Technology

RTAB-Map is an advanced graph-based SLAM framework designed for robust long-term and large-scale mapping in dynamic environments. Key capabilities include:

### Core Features

- **Graph-Based SLAM Architecture**: Implements a sophisticated pose-graph representation where nodes represent keyframes and edges represent spatial transformations between them.
- **Loop Closure Detection**: Utilizes a hierarchical Bag-of-Words (BoW) approach with SURF/SIFT features to recognize previously visited locations, enabling precise drift correction.
- **Multi-Sensor Fusion**: Seamlessly integrates data from RGB-D cameras, LiDAR sensors, stereo cameras, and IMU sensors for enhanced mapping accuracy.
- **Memory-Efficient Processing**: Employs a novel memory management system that intelligently handles large-scale environments by keeping only relevant map sections in active memory.

### LiDAR Integration

Although RTAB-Map is primarily known for visual SLAM, our implementation leverages LiDAR sensors to:
- Provide accurate depth measurements in environments with challenging lighting conditions
- Enhance feature detection in visually homogeneous environments
- Improve mapping precision with high-resolution point clouds
- Enable reliable localization in low-texture scenarios

The fusion of RGB-D and LiDAR data is performed using RTAB-Map's multi-sensor registration capabilities, resulting in more accurate and complete environmental maps than either sensor could provide independently.

### Technical Capabilities

- **Online & Offline Processing**: Supports both real-time SLAM during operation and post-processing refinement for map optimization.
- **3D Point Cloud Generation**: Creates dense, colorized point clouds for detailed environmental modeling.
- **6-DoF Pose Estimation**: Provides accurate six-degree-of-freedom localization crucial for complex navigation tasks.
- **ROS Integration**: Offers comprehensive compatibility with ROS 1 and ROS 2 ecosystems, facilitating easy integration with existing robotic workflows.

### Loop Closure and Navigation in Detail

#### Loop Closure Detection

Loop closure is a critical component of RTAB-Map that corrects accumulated drift and ensures map consistency. The process works as follows:

1. **Feature Extraction**: Visual features (typically SURF or SIFT) are extracted from keyframes and converted into a Bag-of-Words representation.
2. **Vocabulary Matching**: Each new frame is compared against a vocabulary tree built from previously observed features.
3. **Candidate Selection**: Potential loop closure candidates are identified based on feature similarity scores.
4. **Geometric Verification**: 
   - RANSAC-based verification validates spatial consistency between matched frames.
   - 3D-to-3D correspondence establishes transformation between current and past poses.
   - Outlier rejection filtering removes false positives.
5. **Pose Graph Optimization**: 
   - When a loop closure is detected, a constraint is added to the pose graph.
   - The graph is optimized using g2o or GTSAM optimization frameworks.
   - The entire map is adjusted to maintain global consistency.
6. **Adaptive Thresholding**: 
   - The system dynamically adjusts matching thresholds based on environment characteristics.
   - Parameters like `Rtabmap/LoopClosureReextractFeatures` and `Rtabmap/LoopClosureTimeThreshold` fine-tune the process.

Our implementation enhances loop closure by incorporating LiDAR scan matching alongside visual feature matching, providing redundant verification and improving reliability in feature-poor environments.

#### Navigation Framework

RTAB-Map's outputs are tightly integrated with the Navigation2 (Nav2) stack, providing:

1. **Global Planning**:
   - The RTAB-Map generated map is used to create a global costmap.
   - Nav2's global planner (A*, Dijkstra, or NavFn) computes optimal paths through the environment.
   - Map updates from RTAB-Map are dynamically incorporated into planning.
2. **Local Planning**:
   - TEB (Timed Elastic Band) and DWB (Dynamic Window Approach) planners use RTAB-Map's local point clouds.
   - Proximity information from both visual features and LiDAR is fused into the local costmap.
   - Dynamic obstacle avoidance leverages real-time sensor data.
3. **Recovery Behaviors**:
   - When localization confidence decreases, RTAB-Map triggers specific recovery behaviors.
   - These include rotating in place to gather more features or backtracking to known locations.
   - The `Rtabmap/RecoveryMaxDistance` parameter controls when recovery is initiated.
4. **Localization Confidence Metrics**:
   - Inlier feature count provides a measure of registration quality.
   - Covariance matrices estimate pose uncertainty.
   - LiDAR ICP (Iterative Closest Point) matching scores complement visual confidence.
5. **Adaptive Navigation Parameters**:
   - Path planning parameters adjust based on localization confidence.
   - Lower confidence triggers more conservative navigation behaviors.
   - Hybrid driving modes combine reactive and map-based navigation strategies.



## Software Prerequisites
- **ROS 2 Humble Hawksbill**: Full desktop installation recommended
- **Navigation2 Stack**: Complete Nav2 framework for autonomous navigation
- **Gazebo Classic**: Version 11.10.0 or newer for simulation
- **RViz2**: For visualization and monitoring
- **RTAB-Map ROS Package**: Version 0.21.0 or newer

## Installation

### 1. Set Up Your ROS 2 Workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### 2. Clone the Repository

```bash
git clone https://github.com/yourusername/autonomous_initial_localization.git
```

### 3. Install RTAB-Map ROS Package

For proper installation of RTAB-Map in ROS 2 Humble, follow the official instructions from the RTAB-Map repository:

```bash
cd ~/ros2_ws/src
git clone https://github.com/introlab/rtabmap.git
git clone --branch ros2 https://github.com/introlab/rtabmap_ros.git
cd ..
```

For more information and detailed installation instructions, visit the official RTAB-Map ROS repository: https://github.com/introlab/rtabmap_ros/tree/ros2#rtabmap_ros

### 4. Install Additional Dependencies

```bash
cd ~/ros2_ws
sudo apt update
sudo apt install -y ros-humble-nav2-bringup ros-humble-slam-toolbox \
                    ros-humble-gazebo-ros-pkgs ros-humble-robot-localization \
                    ros-humble-laser-filters
rosdep install --from-paths src --ignore-src -r -y
```

### 5. Build the Workspace

```bash
cd ~/ros2_ws
colcon build
source ~/ros2_ws/install/setup.bash
```

## Usage

### Mapping Mode

Mapping mode allows the robot to autonomously explore and build a comprehensive visual SLAM map of the environment.

#### Launch Mapping

```bash
ros2 launch autonomous_initial_localization rtab_tortoise.launch.py
```

#### Configuration Options

- **Resolution**: Adjust map resolution with the `map_resolution:=0.05` parameter (default: 0.05m).
- **Custom Parameters**: Override RTAB-Map parameters using the `params_file:=/path/to/params.yaml` option.
- **LiDAR Configuration**: Set LiDAR parameters with `lidar_enabled:=true lidar_range:=10.0`.

#### Map Management

Generated maps are stored in `~/.ros/rtabmap.db` by default. To specify a custom location:

```bash
ros2 launch autonomous_initial_localization rtab_tortoise.launch.py 
```

### Localization Mode

Once mapping is complete, the saved map is used for efficient localization within the environment.

#### Launch Localization

```bash
ros2 launch autonomous_initial_localization rtab_tortoise.launch.py localization:=True 
```

## System Architecture

The system architecture is modular and integrates several key components:

### Core Components

- **RTAB-Map Node**: Central component managing visual SLAM processes, the map database, and localization.
- **Camera Interface**: Handles RGB-D sensor data acquisition and preprocessing.
- **LiDAR Processing**: Filters and processes LiDAR scans for point cloud registration.
- **Odometry Estimator**: Fuses wheel odometry with visual and LiDAR odometry for robust pose estimation.
- **Map Server**: Provides map data to navigation components.
- **Navigation Stack**: Leverages Nav2 for path planning and obstacle avoidance.

### Data Flow

1. **Sensor Data Acquisition**: RGB-D images, LiDAR scans, and wheel odometry are collected from robot sensors.
2. **Multi-Modal Processing**: Visual features and LiDAR point clouds are processed in parallel.
3. **Sensor Fusion**: Data from different sensors are aligned and fused using calibrated transformations.
4. **Loop Closure Detection**: Previously visited locations are identified to correct accumulated drift.
5. **Pose Graph Optimization**: The map structure is continuously refined for accuracy.
6. **Localization Output**: Precise 6-DoF pose estimates are published to the ROS network.

### Integration Diagram

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

## Performance Considerations

### Optimization Guidelines

- **Processing Requirements**: RTAB-Map with LiDAR integration is computationally intensive. Consider using a dedicated GPU for real-time operation on large maps.
- **Memory Usage**: Large environments may require 8+ GB of RAM. Monitor memory consumption during extended mapping sessions.
- **Camera Calibration**: Proper RGB-D camera calibration is essential for accurate mapping. Use the provided calibration utilities.
- **LiDAR-Camera Calibration**: The extrinsic calibration between the LiDAR and camera is critical for proper fusion. Use the `lidar_camera_calibration` tool included in the package.
- **Loop Closure Frequency**: Adjust the loop closure detection parameters based on environment characteristics:
  - `Rtabmap/DetectionRate`: 2.0 for feature-rich environments, 1.0 for sparse environments.
  - `Rtabmap/MemoryThr`: 50-100 for large areas, 30-50 for smaller spaces.

### Real-World vs. Simulation

- Lighting conditions affect feature extraction quality in real-world deployments.
- Sensor noise requires additional filtering on physical hardware.
- Processing latency impacts real-time operation more significantly on robot hardware.
- LiDAR reflectivity issues may affect point cloud quality on certain surfaces.

## Troubleshooting

### Common Issues

#### Poor Localization Accuracy
- Ensure sufficient visual features in the environment.
- Verify camera and LiDAR calibration parameters.
- Adjust feature extraction parameters in the configuration file.
- Check LiDAR scan filtering parameters.

#### High CPU/Memory Usage
- Reduce the map resolution parameter.
- Decrease the RGB-D image processing rate.
- Enable downsampling of point clouds.
- Adjust the LiDAR scan rate.

#### Database Corruption
- Always use the backup database created automatically in `~/.ros/rtabmap.db.backup`.
- Ensure proper shutdown of the RTAB-Map node before terminating.

#### Build Issues
- If you encounter problems with `colcon build --symlink-install`, try using the standard build command instead:
  ```bash
  colcon build
  ```
- This creates a full copy rather than symbolic links, which can resolve certain path-related issues.

#### Simulation Launch Failures
- If Gazebo or RViz fail to launch properly, first try terminating the process with Ctrl+C and restarting.
- For persistent launch issues, kill any remaining processes:
  ```bash
  pkill -9 -f gz
  pkill -9 -f ros2
  ```
- Then attempt to relaunch the simulation.
- Ensure X11 forwarding is properly configured when launching visualizations over SSH.

### Diagnostic Tools

- **Visualization**: Run the built-in visualizer with:
  ```bash
  ros2 run rtabmap_ros rtabmap --visualize
  ```
- **TF Tree Monitoring**: Check sensor fusion quality with:
  ```bash
  ros2 run rqt_tf_tree rqt_tf_tree
  ```
- **Topic Monitoring**: Inspect data flow with:
  ```bash
  ros2 topic list
  ros2 topic echo /rtabmap/odom
  ```
