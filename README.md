# 🚀 MoveIt Tutorials Implementation in ROS 2 Humble

This repository contains my implementation of the official **MoveIt tutorials**, adapted for a **ROS 2 Humble** environment.  
The goal was to follow the core concepts from the MoveIt documentation while applying several modifications needed for my setup, workspace structure, and robot configuration.

---

## ✅ What I Completed
I went through the major components of the original MoveIt tutorials, including:

- ✅ RobotModel & RobotState  
- ✅ Kinematics and FK/IK examples  
- ✅ PlanningScene and collision environment  
- ✅ PlanningSceneMonitor for maintaining an up-to-date world  
- ✅ MoveGroup interfaces (planning, executing, querying the robot)  
- ✅ PlanningSceneInterface for adding collision objects  
- ✅ Basic motion planning pipelines  
- ✅ ROS 2 nodes, parameters, and execution using `rclcpp`  

All examples were rebuilt and tested directly inside a **ROS 2 Humble** environment.

---

## 🔧 Changes and Adjustments I Made
While following the tutorials, I made several adaptations to match the ROS 2 ecosystem and my specific setup:

- Updated API changes between ROS 1 and ROS 2  
- Adjusted include paths, naming differences, and namespaces  
- Converted example nodes to **`rclcpp`**  
- Modified launch files to ROS 2 conventions  
- Reorganized packages to fit my workspace structure  
- Integrated my own robot configuration and URDF where required  
- Applied differences in PlanningSceneMonitor and TF handling in ROS 2  
- Ensured compatibility with MoveIt for Humble (`moveit2`)  

---

## 📦 ROS 2 + MoveIt 2 Environment
All work was done using:

- **ROS 2 Humble Hawksbill**
- **MoveIt 2 (Humble)**  
- Colcon-based workspace  
- RViz2 for planning visualization  
- Custom URDF and SRDF for testing  

---

## 🎯 Purpose of This Repository
This repository serves as:

- A reference for others learning **MoveIt 2 on ROS 2 Humble**
- A personal record of the steps I completed
- A base for future robotics and motion planning experiments
- A simplified version of MoveIt’s tutorials adapted to real-world usage

---

