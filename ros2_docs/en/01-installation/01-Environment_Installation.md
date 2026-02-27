# Environment Dependencies

Before starting, please ensure that your system meets the following requirements:

- **Operating System**: Ubuntu 22.04 LTS
- **ROS2 Distribution**: Humble
- **Gazebo**: For physical simulation
- **MoveIt2**: For path planning
- **Other dependent packages**: Such as `ros2-control`, `ros2-controllers`, plugins, etc.
- **Python**: 3.10


# Environment Installation Steps

## 1. Install ROS2 Humble

If you have not installed ROS2 Humble yet, please refer to the [ROS2 official documentation](https://docs.ros.org/en/humble/Installation.html).

## 2. Install Gazebo

To support physical simulation, you need to install Gazebo and its related ROS2 packages:

```bash
sudo apt install ros-humble-gazebo-ros-pkgs
```

## 3. Install MoveIt2

Install MoveIt2 to achieve path planning functionality:

```bash
sudo apt install ros-humble-moveit*
```

## 4. Install Controller Related Packages

Install ROS2 control and controller packages to interact with the robotic arm:

```bash
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers
```

## 5. Install Plugins

Install some additional plugins to enhance functionality:

```bash
sudo apt install ros-humble-joint-state-publisher-gui
sudo apt install ros-humble-gazebo-ros2-control
# sudo apt install ros-humble-tf-transformations
```

