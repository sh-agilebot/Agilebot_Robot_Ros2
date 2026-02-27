# 环境依赖

在开始之前，请确保您的系统满足以下要求：

- **操作系统**: Ubuntu 22.04 LTS
- **ROS2 Distribution**: Humble
- **Gazebo**: 用于物理仿真
- **MoveIt2**: 用于路径规划
- **其他依赖包**：如 `ros2-control`, `ros2-controllers`, 插件等
- **Python**: 3.10

# 环境安装步骤

## 1. 安装 ROS2 Humble

如果您尚未安装ROS2 Humble，请执行以下命令进行安装：

```bash
wget http://fishros.com/install -O fishros && . fishros
```

> 注意：此脚本会自动为您安装ROS2 Humble版本。如果需要手动安装，请参考[官方文档](https://docs.ros.org/en/humble/Installation.html)。

## 2. 安装 Gazebo

为了支持物理仿真，您需要安装Gazebo及其相关的ROS2包：

```bash
sudo apt install ros-humble-gazebo-ros-pkgs
```

## 3. 安装 MoveIt2

安装MoveIt2以实现路径规划功能：

```bash
sudo apt install ros-humble-moveit*
```

## 4. 安装控制器相关包

安装ROS2控制和控制器包，以便与机械臂交互：

```bash
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers
```

## 5. 安装插件

安装一些额外的插件来增强功能：

```bash
sudo apt install ros-humble-joint-state-publisher-gui
sudo apt install ros-humble-gazebo-ros2-control
# sudo apt install ros-humble-tf-transformations
```