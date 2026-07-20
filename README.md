# NeuraLara8

**Overview**
This repository contains the simulation environment for the ROS 2 Humble dual-arm robotic manipulation system. Please note that this codebase represents only the simulation portion of the completed master's thesis *"Control Methods for a Dual-Arm Robotic Manipulation System"*. The full thesis document, along with media and videos demonstrating the various executed tasks (such as collision avoidance and visual servoing), have been included in this repository for complete reference.

---

##Workspace setup

Create an image and then the container using `ros2_lara8_ws` as the workspace, then build and source

# Start the simulation
Launch MoveIt!
```bash
ros2 launch moveit_lara8_moveit_config dual_demo.launch.py
```

# Gazebo+MoveIt! simulation
Launch MoveIt! with Gazebo 
```bash
ros2 launch moveit_lara8_moveit_config gazebo.launch.py
```






