# NeuraLara8
NeuraLara8 ROS2 Humble

Create an image 
```bash
./docker_build_image.sh [IMAGE_NAME] 
```
and then the container
```bash
./docker_run_container.sh [IMAGE_NAME] [CONTAINER_NAME] ros2_lara8_ws
```

Then build and source

# Start the simulation
Launch MoveIt!
```bash
ros2 launch moveit_lara8_moveit_config dual_demo.launch.py
```

# Gazebo+MoveIt! simulation
Launch MoveIt! with Gazebo 
```bash
ros2 launch moveit_lara8_moveit_config gazebo_launch.py
```






