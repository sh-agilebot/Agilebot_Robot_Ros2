# 在RViz中显示捷勃特机器人模型

本章节介绍如何使用 ROS 2 在 RViz 中显示捷勃特机器人模型，并通过 GUI 工具进行交互。RViz（ROS 可视化）是 ROS 生态中用于实时可视化机器人模型、传感器数据和环境信息的工具。它可以：

* 加载并显示 URDF 格式的机器人模型
* 可视化激光雷达点云、深度相机点云等 2D/3D 传感器数据
* 显示机器人关节状态、轨迹规划结果和 TF 坐标变换
* 通过交互标记和 GUI 插件，实现对机器人姿态和目标的在线调整

显示效果如下：



![RViz](/assets/rviz.png)
---

要查看上海捷勃特机器人的URDF模型，请执行以下命令启动RViz：

```bash
cd {您的工作目录}
source install/setup.bash
ros2 launch gbt_description display_robot.launch.py robot_type:=<机器人型号>
```

例如，如果您想查看C5A型号的机器人：

```bash
cd {您的工作目录}
source install/setup.bash
ros2 launch gbt_description display_robot.launch.py robot_type:=C5A
```

其他型号的机器人启动方式类似：

```bash
# C7A 机器人
ros2 launch gbt_description display_robot.launch.py robot_type:=C7A

# C12A 机器人
ros2 launch gbt_description display_robot.launch.py robot_type:=C12A

# C16A 机器人
ros2 launch gbt_description display_robot.launch.py robot_type:=C16A
```

目前支持以下协作机器人型号：
- C5A
- C7A
- C12A
- C16A



> 1.工作目录是指包含src 和install文件夹的目录，例如：/home/{你的用户名}/ws
这将在RViz中加载指定型号机器人的URDF模型。
>
>2.更多机器人型号请访问官网查询，https://www.sh-agilebot.com