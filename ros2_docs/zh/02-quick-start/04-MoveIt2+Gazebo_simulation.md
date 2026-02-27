
# Moveit2+Gazebo联合仿真

本章节介绍如何使用Moveit2和Gazebo进行捷勃特机器人的联合仿真。首先使用moveit2进行运动规划，然后将规划路径通过action方式发送给Gazebo进行仿真。

![Moveit2+Gazebo联合仿真](/assets/moveit_gazebo.png)

您可以结合Moveit2和Gazebo来实现更复杂的捷勃特机器人仿真场景。请按照以下步骤操作：

## 终端1：启动Gazebo仿真

首先，在一个终端窗口中启动Gazebo仿真：

```bash
ros2 launch {机器人型号}_moveit_config gazebo.launch.py
```

例如：

```bash
ros2 launch c5a_moveit_config gazebo.launch.py
```

## 终端2：启动Moveit2和RViz

然后，在另一个终端窗口中启动Moveit2和RViz：

```bash
ros2 launch {机器人型号}_moveit_config gazebo_moveit_rviz.launch.py
```

例如：

```bash
ros2 launch c5a_moveit_config gazebo_moveit_rviz.launch.py
```

这将同时启动Gazebo仿真和Moveit2的RViz界面，使您能够在虚拟环境中对机器人进行运动规划。

## 支持的机器人型号

当前支持以下协作机器人型号的联合仿真：

- **C5A** - 5公斤负载协作机器人
- **C7A** - 7公斤负载协作机器人
- **C12A** - 12公斤负载协作机器人
- **C16A** - 16公斤负载协作机器人

使用时只需将命令中的 `{机器人型号}` 替换为对应的型号即可，例如：

```bash
# C7A 机器人
ros2 launch c7a_moveit_config gazebo.launch.py
ros2 launch c7a_moveit_config gazebo_moveit_rviz.launch.py

# C12A 机器人
ros2 launch c12a_moveit_config gazebo.launch.py
ros2 launch c12a_moveit_config gazebo_moveit_rviz.launch.py

# C16A 机器人
ros2 launch c16a_moveit_config gazebo.launch.py
ros2 launch c16a_moveit_config gazebo_moveit_rviz.launch.py
```