# Autonomous Mobile Manipulation with ROS 2 Humble

This project integrates a mobile robot (`fra2mo`) and a robotic manipulator (`iiwa`) in a Gazebo simulation. The system performs autonomous exploration, ArUco tag detection, robust navigation using Nav2, and visual servoing control.

## 📦 Dependencies
Ensure you have the following packages in your workspace `src` folder:
* `ros2_fra2mo` 
* `ros2_iiwa`
* `ros2_kdl_package`
* `aruco_ros` (External)
* `m-explore-ros2` (External)

---

## 🚀 How to Run

### Part 1: Exploration & Target Detection
In this phase, the robot autonomously explores the unknown environment using SLAM and detects the KUKA iiwa manipulator position.

### 1. Launch the Simulation
Load the world, robots, and ArUco detector:
```bash
ros2 launch ros2_fra2mo project.launch.py
```
### 2. Start Autonomous Exploration
Launch the explore_lite node to auto-navigate and map the area:
```bash
ros2 launch ros2_fra2mo fra2mo_explore.launch.py
```
### 3. Save Target Position
Run the logic node to detect the ArUco tag, transform coordinates, and save the goal to YAML:
```bash
ros2 run ros2_fra2mo save_iiwa_pose.py
```
### 4. Save the explored map
```bash
ros2 run nav2_map_server map_saver_cli -f map
```

---

### Part 2: Navigation & Visual Servoing
In this phase, the robot localizes within the generated map, navigates to the saved target, and the manipulator performs visual servoing.

### 1. Launch Simulation (Velocity Mode)
Restart the simulation with the velocity controller interface required for visual servoing:
```bash
ros2 launch ros2_fra2mo project.launch.py iiwa_controller:=velocity_controller command_interface:=velocity
```

### 2. Start Localization (AMCL)
Initialize the Adaptive Monte Carlo Localization system:
```bash
ros2 launch ros2_fra2mo fra2mo_amcl.launch.py
```

### 3. Start Navigation Stack (Nav2)
Launch the navigation server and costmaps:
```bash
ros2 launch ros2_fra2mo fra2mo_navigation.launch.py
```
### 4. Navigate to Target
Send fra2mo to the previously saved iiwa coordinates (with safety offset):
```bash
ros2 run ros2_fra2mo go_too_iiwa.py
```
### 5. Start Manipulator Vision Control
Activate the visual servoing on the iiwa arm to track the package:
```bash
ros2 run ros2_kdl_package ros2_kdl_node --ros-args -p cmd_interface:=velocity -p ctrl:=vision_ctrl
```
