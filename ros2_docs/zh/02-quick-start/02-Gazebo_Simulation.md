# 在Gazebo中仿真机器人

本章节介绍如何在 Gazebo 中模拟捷勃特机器人——Gazebo 是一款高度可扩展的 3D 机器人仿真器，支持多物理引擎（ODE、Bullet、DART）、丰富的传感器插件（LiDAR、深度相机、IMU、GPS），并通过 gazebo_ros_pkgs 与 ROS 2 实现深度集成。Gazebo 可让您在真实感虚拟环境中安全地测试和优化算法，然后再部署到真实硬件上。

![](/assets/gazebo.png)
---
为了在Gazebo中模拟捷勃特机器人，请使用以下命令：

```bash
ros2 launch gbt_gazebo gazebo_demo.launch.py robot_type:=<机器人型号> [controller_name:=<控制器名称>]
```

例如，启动C5A型号的Gazebo仿真：

```bash
ros2 launch gbt_gazebo gazebo_demo.launch.py robot_type:=C5A controller_name:=gbt_c5a_arm_controller
```

其他型号的机器人启动方式类似：

```bash
# C7A 机器人
ros2 launch gbt_gazebo gazebo_demo.launch.py robot_type:=C7A

# C12A 机器人
ros2 launch gbt_gazebo gazebo_demo.launch.py robot_type:=C12A

# C16A 机器人
ros2 launch gbt_gazebo gazebo_demo.launch.py robot_type:=C16A
```

这将打开Gazebo，并加载相应的机器人模型和环境设置。

>控制器名称通常为 `gbt_{机器人型号}_arm_controller`，具体名称请参考 `gbt_moveit` 包对应机器人的控制器配置文件。
>
>比如：`gbt_moveit_config/c5a_moveit_config/config/ros2_controllers.yaml`
>
> `controller_name`参数可以不填，将使用默认规则自动拼接，即`gbt_{robot_type.lower()}_arm_controller`。




## 配置与自定义

可以通过修改 `config` 目录中的文件来自定义 Gazebo 仿真环境，包括调整物理参数、添加或删除对象等。

## 注意事项

若gazebo没有正常关闭，请使用以下命令关闭gazebo：
```bash
kill -9 $(ps aux | grep '[g]azebo' | awk '{print $2}')
```