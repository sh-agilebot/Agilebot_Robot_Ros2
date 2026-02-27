# 将机器人姿态同步到RViz中显示

当使用 MoveIt2 或 Gazebo 进行路径规划和仿真时，通常需要获取物理机器人的**实时位姿**作为参考。此功能**实时采集**机器人的关节状态数据，并将其发布到 `/joint_states` 话题，从而实现机器人真实位姿的同步。发布到该话题的数据可被 **MoveIt2、Gazebo 和 RViz 等工具订阅并用于** **显示**当前机器人状态、**规划**路径或**驱动**仿真。

## 前置条件

在开始之前，请确保：

1. 已完成 [环境安装与项目构建](../01-installation/index.md)
2. 已安装 Agilebot Robot SDK（参见 [构建项目](../01-installation/02-build.md)）
3. 机器人控制器已上电并连接到网络

## 设置机器人IP

编辑 `gbt_driver/config/robot_config.yaml` 文件，设置 `robot_ip_address` 字段为实际的机器人IP地址：

```bash
vi gbt_driver/config/robot_config.yaml
```

```yaml
robot_ip_address: "192.168.x.x"  # 修改为实际机器人IP
```

## 启动机器人状态同步

使用以下命令启动RViz并同步显示机械臂状态：

```bash
source install/setup.bash
ros2 launch gbt_driver gbt_bridge.launch.py robot_type:=<机器人型号>
```

### 支持的机器人型号

- `C5A` - 5公斤负载协作机器人
- `C7A` - 7公斤负载协作机器人
- `C12A` - 12公斤负载协作机器人
- `C16A` - 16公斤负载协作机器人

### 示例

对于C5A型号：

```bash
source install/setup.bash
ros2 launch gbt_driver gbt_bridge.launch.py robot_type:=C5A
```

其他型号的机器人启动方式类似：

```bash
# C7A 机器人
ros2 launch gbt_driver gbt_bridge.launch.py robot_type:=C7A

# C12A 机器人
ros2 launch gbt_driver gbt_bridge.launch.py robot_type:=C12A

# C16A 机器人
ros2 launch gbt_driver gbt_bridge.launch.py robot_type:=C16A
```

## 验证同步状态

### 方法1：查看RViz显示

启动后，RViz窗口会自动打开，您可以看到：
- 机器人的URDF模型
- 实时更新的关节状态

### 方法2：查看话题数据

在另一个终端中查看 `/joint_states` 话题：

```bash
source install/setup.bash
ros2 topic echo /joint_states
```

您应该能看到类似以下输出：

```
header:
  stamp:
    sec: 1234567890
    nanosec: 123456789
  frame_id: ''
name:
- joint1
- joint2
- joint3
- joint4
- joint5
- joint6
position:
- 0.0
- 0.1
- 0.2
- 0.3
- 0.4
- 0.5
...
```

### 方法3：查看详细反馈状态（可选）

如需获取更详细的机器人状态信息（包括报警码、伺服状态等），可以额外启动 `robot_status` 节点：

```bash
source install/setup.bash
ros2 launch gbt_driver gbt_feedback.launch.py
```

然后查看 `/gbt_driver/feedback_states` 话题：

```bash
ros2 topic echo /gbt_driver/feedback_states
```

## 故障排查

### 无法连接到机器人

1. 检查机器人IP地址是否正确配置
2. 检查网络连接：`ping <机器人IP>`
3. 确认机器人控制器已上电

### RViz中机器人姿态不更新

1. 检查 `/joint_states` 话题是否正常发布：
   ```bash
   ros2 topic hz /joint_states
   ```
2. 确认机器人处于可操作状态（非报警状态）

### 模块导入错误

如遇 `ModuleNotFoundError: No module named 'Agilebot'` 错误，请确保已安装SDK：

```bash
pip install Agilebot.Robot.SDK.A-*.whl
```
