# as_drone_ws24
# TUM - Autonomous Systems

### Group 12
- Xuanyi Lin          (xuanyi.lin@tum.de)
    - mapping_pkg (PCD, OctoMap), pathplanning_pkg
- Jiaxiang Yang       (jiaxiang.yang@tum.de)
    - mapping_pkg (PCD, OctoMap), trajectory_generator
- Yicun Song          (go25sem@mytum.de)
    - pathplanning_pkg
- Enze Wang           (enze.wang@tum.de)
    - trajectory_generator
- Mohammadali Rahmati (Mohammadali.rahmati@tum.de)
    - state_machine_pkg, light_detector_pkg

### Table of contents
1.[Introduction](#introduction)
  

## 1. Introduction <a name="introduction"></a>

This repository provides ROS packages for autonomous drone exploration in a Unity cave environment. Using a depth camera and OctoMap, the drone generates a 3D Voxel Grid, detects unexplored areas, and plans paths via RRT*. The paths are converted into executable trajectories using mav_trajectory_generation, enabling efficient navigation. Additionally, a semantic camera detects and locates four target objects (lanterns).

### 1.1. Reused external libraries <a name="reused_externel_libraries"></a>

- `Depth Image Proc` (https://wiki.ros.org/depth_image_proc)

- `OctoMap` (https://github.com/OctoMap/octomap)

- `Point Cloud Library (PCL)` (https://github.com/PointCloudLibrary/pcl)

- `Pcl-Optics` (https://github.com/Nandite/Pcl-Optics/tree/master)

- `Open Motion Planning Library (OMPL)` (https://github.com/ompl/ompl)

- `Flexible Cloud Library (FCL)` (https://github.com/flexible-collision-library/fcl)

- `OctomapPlanner` (https://github.com/ArduPilot/OctomapPlanner)
    
- `MAV Trajectory Generation` (https://github.com/ethz-asl/mav_trajectory_generation)

### 1.2. own packages <a name="own_packges"></a>
- `state_machine_pkg` 
- `pathplanning_pkg` 
- `trajectory_generator` 
- `light_detector_pkg` 

## 2. Installation guide <a name="installation_guide"></a>

### 2.1. System requirement <a name="system_requirement"></a>

 - `Ubuntu 20.04` setup guide link: https://releases.ubuntu.com/focal/ 
 - `ROS Noetic` setup guide link: https://wiki.ros.org/noetic/Installation/Ubuntu 

### 2.2. Installing dependencies <a name="installing_dependencies"></a>

updating
```
sudo apt-get update
sudo apt-get upgrade
```

installing requireded basic tools  . 
```
sudo apt install git wstool wget libtool apt-utils python3-catkin-tools
```

installing `depth_image_proc` package
```
sudo apt install ros-noetic-depth-image-proc
```

installing `octomap` and `octomap_mapping` packages
```
sudo apt-get install ros-noetic-octomap ros-noetic-octomap-mapping
```

installing `Open Motion Planning Library (OMPL)` and `Flexible Cloud Library (FCL)`
```
sudo apt-get install ros-noetic-ompl ros-noetic-fcl
```

### 2.3. Building project <a name="building_project"></a>
Open a terminal and clone the project repository (branch: drone)
```
git clone git@github.com:zoey345/as_drone_ws24.git
```
Download simulation.zip from the folowing link.

- https://syncandshare.lrz.de/getlink/fi7Vw11aA5WwyMBRPVQYun/

Copy the content \Simulation_Data\sharedassets0.assets.resS into 

```
as_drone_ws24/catkin_ws/src/simulation/Simulation_Data
```
Set the `Simulation.x86_64` as executable
```
cd as_drone_ws24/catkin_ws/src/simulation
chmod +x Simulation.x86_64 
```
Build the project
```
cd ../..
catkin build
```

## 3. Launching the simulation <a name="launching_the_simulation"></a>

running the simulation
```
source devel/setup.bash
roslaunch launch_pkg launch.launch
```
