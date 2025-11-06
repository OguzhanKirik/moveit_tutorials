# 🧭 MoveIt Planning Scene Monitor — Concept Overview

The **PlanningSceneMonitor** is the recommended interface for maintaining an up-to-date planning scene in MoveIt.  
Because MoveIt’s planning environment consists of several interconnected components, it can be confusing at first.  
This guide explains how **RobotState**, **CurrentStateMonitor**, **PlanningScene**, **PlanningSceneMonitor**, and **PlanningSceneInterface** relate to each other.

---

## ✅ RobotState
A **RobotState** is a **snapshot** of the robot at a specific moment in time.

It includes:
- The **RobotModel**
- A set of **joint values**
- Utility methods for forward/inverse kinematics, transforms, and collision checking

A `RobotState` does **not** update automatically — it must be manually modified or replaced.

---

## ✅ CurrentStateMonitor (CSM)
The **CurrentStateMonitor** is a ROS wrapper around a `RobotState`.

It:
- Subscribes to **JointState** messages
- Updates its internal `RobotState` in real time
- Keeps the robot’s state synchronized with incoming sensor data

Think of it as a live, real-time updater for the robot’s current state.

---

## ✅ PlanningScene
A **PlanningScene** is a snapshot of the **entire world** relevant to motion planning.

It includes:
- A **RobotState**
- Collision objects
- Allowed collision matrices
- Environment geometry

A `PlanningScene` can be used for:
- Collision checking
- Evaluating robot trajectories
- Querying the environment

It does **not** automatically stay up-to-date.

---

## ✅ PlanningSceneMonitor
The **PlanningSceneMonitor** wraps a `PlanningScene` and provides ROS interfaces to keep it synchronized.

It handles:
- Robot state updates (via `CurrentStateMonitor`)
- Collision object updates
- Environment/world changes
- Planning scene diffs published by MoveGroup

In short:  
👉 The PlanningSceneMonitor keeps the robot and the environment **continuously up-to-date**.

---

## ✅ PlanningSceneInterface
The **PlanningSceneInterface** offers a simple C++ API for modifying the planning scene *without* manually handling ROS topics or services.

It allows you to:
- Add and remove collision objects
- Update object poses
- Attach and detach objects to the robot

This is the recommended way for user-level applications to interact with MoveIt’s planning scene.

---

## ✅ Summary Diagram (Conceptual)

