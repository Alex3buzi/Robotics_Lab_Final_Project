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

**1. Launch the Simulation**
Load the world, robots, and ArUco detector:
```bash
ros2 launch ros2_fra2mo project.launch.py
```
**2. Start Autonomous Exploration**
Launch the explore_lite node to auto-navigate and map the area:
```bash
ros2 launch ros2_fra2mo fra2mo_explore.launch.py
```
