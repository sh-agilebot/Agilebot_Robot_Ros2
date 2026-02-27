# 使用MoveIt2进行机器人运动规划
本章节介绍如何使用 MoveIt 2 对捷勃特机器人进行运动规划。MoveIt 2 是基于 ROS 2 平台的机器人运动规划框架，集成了以下核心功能：

* **碰撞检测**：在运动规划过程中实时避让环境和自身碰撞
* **运动学求解**：提供高效的正、逆运动学解算器
* **路径规划**：支持多种平面和空间路径规划算法（如 OMPL、CHOMP、STOMP 等）
* **轨迹优化**：对生成的轨迹进行平滑和速度/加速度限幅
* **轨迹执行**：与硬件控制器无缝对接，实现实时运动控制
* **实时重规划**：在动态环境或目标变化时，可自动更新规划结果
* **可视化与调试**：通过 RViz 插件展示规划过程，支持交互式目标设置与调参

借助 MoveIt 2，您可以在仿真或真实机器人上快速生成并执行高质量的臂部运动路径，从而简化开发与调试流程。


![](/assets/moveit.png)

---
要使用MoveIt2进行上海捷勃特机器人产品进行路径规划和运动控制，请运行以下命令：

```bash
ros2 launch {机器人型号}_moveit_config demo.launch.py
```

例如，启动C5A型号的MoveIt2配置：

```bash
ros2 launch c5a_moveit_config demo.launch.py
```

其他型号的机器人启动方式类似：

```bash
# C7A 机器人
ros2 launch c7a_moveit_config demo.launch.py

# C12A 机器人
ros2 launch c12a_moveit_config demo.launch.py

# C16A 机器人
ros2 launch c16a_moveit_config demo.launch.py
```

这将启动MoveIt2的演示界面，允许您进行路径规划和其他操作。

> Ref: [MoveIt2 official documentation](https://moveit.picknik.ai/main/doc/examples/setup_assistant/setup_assistant_tutorial.html#step-1-start).