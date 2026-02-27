
<div align="right">
  
[中文简体](readme_cn.md)|[English](readme.md)

</div>

[TOC]

<div align="center">

# 上海捷勃特机械臂 ROS2 驱动使用说明 (ROS2) - v0.1.0

## 文件修订记录

| 版本号  | 时间          | 备注     |
| :-----: | :-----------: | :------- |
| N/A | 2024年11月14日 | 初始版本 |
| v0.1.0  | 2025年8月4日 | 提测版本|

</div>

---

## 1. 简介

本文档主要介绍了上海捷勃特机械臂 ROS2 驱动的使用方法。通过该驱动，可以实现：

- 发布机械臂状态信息
- 控制机械臂运动
- 将物理机器人状态同步到urdf
- service server:提供控制运动、IO控制、程序控制、伺服上下电等接口
- action server:moveit2控制物理机械臂
- 接收离线轨迹文件并执行

---

## 2. 基本功能

### 2.1 发布机器人状态信息

#### 消息结构

`gbt_interface/msg/FeedbackState.msg`

以下是机械臂状态信息的消息结构：

```yaml
std_msgs/Header header

# Connection Status
bool is_connected                            # Whether connected to the robot

# Robot Status
string robot_name                            # Robot Name
RobotStatus robot_status                     # Robot Status
ControllerStatus controller_status           # Controller Status
ServoStatus servo_status                     # Servo Status
ArmSoftModeStatus arm_soft_mode_status       # Arm Soft Mode Status
# 注意：机器人操作模式（软模式）在实体机上只能通过手柄钥匙切换，不可以通过页面或者SDK切换

# Robot joint 
sensor_msgs/JointState joint_states          # Robot joint states
geometry_msgs/PoseStamped flange_pose        # Robot flange pose
geometry_msgs/PoseStamped tool_pose          # Robot tool pose

# Alarm code 
AlarmCode[] alarm_code_list                  # Robot alarm codes

float32 speed_percentage                     # Speed percentage

```

#### 设置机械臂 IP 地址

在 `gbt_driver/config/robot_config.yaml` 配置文件中，修改机械臂的 IP 地址。

```yaml
robot_ip_address: "192.168.x.x"  # 请根据实际情况填写机械臂的 IP 地址
```

#### 启动 ROS2 驱动节点

在终端启动 `gbt_driver` 节点：

```bash
source install/setup.bash
ros2 launch gbt_driver gbt_feedback.launch.py
```

#### 获取机械臂状态信息

启动 `gbt_driver` 节点后，驱动会自动连接到机械臂，并开始发布机械臂的状态信息到以下话题：

- `/gbt_driver/feedback_states`

通过订阅该话题，您可以获取以下信息：

- 机械臂连接状态
- 机器人状态
- 控制器状态
- 伺服状态
- 机械臂软模式状态（注意：机器人操作模式在实体机上只能通过手柄钥匙切换，不可以通过页面或者SDK切换）
- 机械臂的关节状态
- 末端执行器的位姿
- 机器人报警码
- 速度百分比

例如，您可以使用以下命令来查看机械臂的状态信息：

```bash
source install/setup.bash
ros2 topic echo /gbt_driver/feedback_states
```

### 2.2 将机械臂的状态同步到urdf

#### 启动 rviz2

在终端启动 `rviz2`：

```bash
source install/setup.bash
ros2 launch gbt_driver gbt_bridge.launch.py
```

> 通过rviz即可同步的显示机械臂的状态

### 2.3 Service Server

#### 移动到点功能

| 功能描述 | 移动到指定位置 |
| :---: | :---- |
| 接口名称 | `gbt_interface/action/MoveToPose` |
| 接口说明 | 该接口用于将机械臂移动到指定的目标位置。 |
| 启动服务 | `ros2 launch gbt_driver gbt_service_server.launch.py` |
| 接口使用示例 | `ros2 action send_goal /gbt_driver/move_to_pose gbt_interface/action/MoveToPose "{ x: 400.0, y: -200.0, z: 500.0, a: 100.0, b: 30.0, c: 110, vel: 1, acc: 1 }"` |
|注意事项|使用笛卡尔坐标系运动 move_joint，vel 和 acc 表示速度和加速度的倍数，取值范围为0到1|

#### IO功能

**设置IO**

| 功能描述 | 设置IO |
| :---: | :---- |
| 接口名称 | `gbt_interface/srv/IO` |
| 接口说明 | 该接口用于设置IO端口的输入输出状态。 |
| 启动服务 | `ros2 launch gbt_driver gbt_service_server.launch.py` |
| 接口使用示例 | `ros2 service call /gbt_driver/service_server/io gbt_interface/srv/IO "{signal_type: 2, signal_port: 1,signal_value: 1,command: set}"` |
|signal_type|可选参数：`DI = 1` `DO = 2` `UI = 3` `UO = 4` `RI = 5` `RO = 6` `GI = 7` `GO = 8`|

**获取IO**

| 功能描述 | 获取IO |
| :---: | :---- |
| 接口名称 | `gbt_interface/srv/IO` |
| 接口说明 | 该接口用于获取指定IO端口的输入输出状态。 |
| 启动服务 | `ros2 launch gbt_driver gbt_service_server.launch.py` |
| 接口使用示例 | `ros2 service call /gbt_driver/service_server/io gbt_interface/srv/IO "{signal_type: 1, signal_port: 1,command: get}"` |
|signal_type|可选参数：`DI = 1` `DO = 2` `UI = 3` `UO = 4` `RI = 5` `RO = 6` `GI = 7` `GO = 8`|

#### 控制程序

| 功能描述 | 控制程序运行 |
| :---: | :---- |
| 接口名称 | `gbt_interface/srv/ProgramControl` |
| 接口说明 | 该接口用于控制程序的启动、暂停或停止等操作。 |
| 启动服务 | `ros2 launch gbt_driver gbt_service_server.launch.py` |
| 接口使用示例 | `ros2 service call /gbt_driver/service_server/program_control gbt_interface/srv/ProgramControl "{command: 'start', program_name: 'test'}"` |
|可选参数|`command`可选参数：`start`、`pause`、`resume`、`stop`|

#### 伺服上下电

| 功能描述 | 伺服上下电控制 |
| :---: | :---- |
| 接口名称 | `gbt_interface/srv/Servo` |
| 接口说明 | 该接口用于控制伺服电机的上下电状态。 |
| 启动服务 | `ros2 launch gbt_driver gbt_service_server.launch.py` |
| 接口使用示例 | `ros2 service call /gbt_driver/service_server/servo_power gbt_interface/srv/Servo "{servo_on: true}"` |
|可选参数|`servo_on`可选参数：`true`、`false`|

### 2.4 发送脚本

| 功能描述 | 发送脚本 |
| :---: | :---- |
| 接口名称 | `gbt_interface/srv/SendScript` |
| 接口说明 | 该接口用于发送脚本到机械臂。 |
| 启动服务 | `ros2 launch gbt_driver gbt_service_server.launch.py` |
| 接口使用示例 | `ros2 service call /gbt_driver/service_server/send_script gbt_interface/srv/SendScript "{script_name: 'test', script_content: 'SUB main\n  MOVEJ PR[1] 100 SD 200.5\n  RETURN\nEND'}"`|

### 2.5 Action Server（Experimental）

**说明：** 将MoveIt算出的轨迹转译成手臂运动的离线轨迹并驱动手臂完成该轨迹
>**注意**： 请在自动模式下进行（机器人操作模式在实体机上只能通过手柄钥匙切换，不可以通过页面或者SDK切换）
**设置机器人IP**
`gbt_driver/config/robot_config.yaml` 中设置 `robot_ip_address` 为机器人IP地址
**终端**：

```bash
source install/setup.bash
ros2 launch gbt_driver gbt_action_server.launch.py <robot_type>:= <机器人型号> interpolate_mode:={quintic or spline}

```

> 可选`quintic` 或者 `spline`，默认使用`quintic`插值。
>`quintic` 是一种五次多项式插值方法，而 `spline` 是一种五次B样条插值方法。

比如：

```bash
ros2 launch gbt_driver gbt_action_server.launch.py robot_type:=C5A interpolate_mode:=quintic
```

### 2.6 离线轨迹

**说明：** 通过离线轨迹文件（csv）控制机械臂运动,可以用来实现复杂且精细的运动。轨迹文件样式参考：[gbt_driver/gbt_driver/test.csv](gbt_driver/test.csv)

#### 轨迹文件说明

离线轨迹文件为CSV格式，表头如下：

```csv
ts,pts_J1,pts_J2,pts_J3,pts_J4,pts_J5,pts_J6,vel_J1,vel_J2,vel_J3,vel_J4,vel_J5,vel_J6,acc_J1,acc_J2,acc_J3,acc_J4,acc_J5,acc_J6,jerk_J1,jerk_J2,jerk_J3,jerk_J4,jerk_J5,jerk_J6,do_port,do_state
```

##### 格式说明

 **1. `ts`（时间戳）**

- **含义**：表示当前轨迹点的时间信息，通常以秒（s）为单位。
- **作用**：用于同步机械臂的运动时间，确保各关节在指定时间点达到目标位置、速度和加速度。

 **2. `pts_J1` 到 `pts_J6`（关节位置）**

- **含义**：
  - `pts_J1` 到 `pts_J6` 分别表示机械臂的 **6个关节的位置（Joint Position）**。
  - 角度（单位：弧度，rad）
- **作用**：
  - 定义机械臂末端执行器的空间位置和姿态（通过正运动学计算）。
  - 是轨迹规划的核心数据，描述机械臂从起点到终点的路径。

 **3. `vel_J1` 到 `vel_J6`（关节速度）**

- **含义**：
  - `vel_J1` 到 `vel_J6` 分别表示机械臂 **6个关节的速度（Joint Velocity）**。
  - 单位是 **弧度/秒（rad/s）**
- **作用**：
  - 控制机械臂各关节的运动速度，确保末端执行器沿规划路径平稳移动。
  - 避免因速度突变导致的机械冲击或振动。

 **4. `acc_J1` 到 `acc_J6`（关节加速度）**

- **含义**：
  - `acc_J1` 到 `acc_J6` 分别表示机械臂 **6个关节的加速度（Joint Acceleration）**。
  - 单位是 **弧度/秒²（rad/s²）**
- **作用**：
  - 描述关节速度的变化率，影响机械臂的动态性能。
  - 过高的加速度可能导致机械臂超载或产生不必要的振动。

 **5. `jerk_J1` 到 `jerk_J6`（关节加加速度）**

- **含义**：
  - `jerk_J1` 到 `jerk_J6` 分别表示机械臂 **6个关节的加加速度（Joint Jerk）**。
  - 单位通常是 **弧度/秒³（rad/s³）**
- **作用**：
  - 描述加速度的变化率，直接影响机械臂运动的平滑性。
  
 **6. `do_port`（数字输出端口）**

- **含义**：
  - 表示机械臂的 **数字输出端口（Digital Output Port）** 编号。
  - 通常用于控制外部设备（如夹爪、吸盘、传感器等）。
- **作用**：
  - 在特定时间点触发外部设备的开关状态（例如夹爪的抓取或释放）。

 **7. `do_state`（数字输出状态）**

- **含义**：
  - 表示数字输出端口（`do_port`）的 **状态（State）**，通常为布尔值（0 或 1）。
  - 例如：
    - `0` 表示关闭（如夹爪松开）。
    - `1` 表示开启（如夹爪闭合）。
- **作用**：
  - 协调机械臂运动与外部设备的联动（例如在机械臂到达目标位置后触发夹爪抓取）。

#### 使用说明

**设置机器人IP**
`gbt_driver/config/robot_config.yaml` 中设置 `robot_ip_address` 为机器人IP地址

##### 启动服务

```bash
source install/setup.bash
ros2 ros2 launch gbt_offline_trajectory.launch.py
```

##### 接口使用示例

[sample.csv](../assets/sample.csv)

```bash
ros2 action send_goal gbt_driver/trajectory gbt_interface/action/OfflineTrajectory "{trajectory_path: '/home/gbt/tmp/action_server_trajectory.csv'}"
```

### 2.7 紧急停止

| 功能描述 | 紧急停止 |
| :---: | :---- |
| 接口名称 | `gbt_interface/srv/EmergencyStop` |
| 接口说明 | 该接口用于控制机械臂的紧急停止状态。 |
| 启动服务 | `ros2 launch gbt_driver gbt_service_server.launch.py` |
| 接口使用示例 | `ros2 service call /gbt_driver/service_server/emergency_stop gbt_interface/srv/EmergencyStop "{}"` |

### 2.8 led开关控制

| 功能描述 | led开关控制 |
| :---: | :---- |
| 接口名称 | `gbt_interface/srv/LED` |
| 接口说明 | 该接口用于控制机械臂的led灯开关状态。 |
| 启动服务 | `ros2 launch gbt_driver gbt_service_server.launch.py` |
| 接口使用示例 | `ros2 service call /gbt_driver/service_server/led gbt_interface/srv/LED "{led_on: true}"` |

---

## 3. 注意事项

- 请确保机械臂的 IP 地址在配置文件中正确设置。
- 启动节点时，确保网络连接正常，确保 ROS2 和机械臂在同一网络下。
- 如遇到连接或通讯问题，请检查防火墙设置和网络配置。

---

## 4. 开源许可

本项目采用BSD 3-Clause License开源许可。
