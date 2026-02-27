
<div align="right">
  
[中文简体](#)|[English](readme.md)

</div>

[TOC]

<div align="center">

# 上海捷勃特机械臂ROS2接口说明-v0.1.0

文件修订记录：

|版本号 | 时间 | 备注 |
| :---: | :---- | :---: |
|v0.0.0 | 2024年11月14日 | 拟制 |
|v0.1.0 | 2025年8月4日 | 发布版本 |

</div>

## 简介

本文档主要介绍上海捷勃特机械臂ros2接口。

## 机械臂基本信息

### 机器人基本信息（msg）

| 功能描述 | 机器人基本信息 |
| :---: | :---- |
|接口名称|gbt_interface/msg/FeedbackState|
|接口说明|包含以下信息：机器人名称、机器人状态、控制器状态、伺服控制器状态、机械臂软模式状态，关节状态和末端位置,报警码，全局速度，已激活的UF/TF|

**详细内容**

```yaml
std_msgs/Header header

# Connection Status
bool is_connected                            # Whether connected to the robot

# Robot Status
string robot_type                            # Robot Type
RobotStatus robot_status                     # Robot Status
ControllerStatus controller_status           # Controller Status
ServoStatus servo_status                     # Servo Status
ArmSoftModeStatus arm_soft_mode_status       # Arm Soft Mode Status

# Robot joint 
sensor_msgs/JointState joint_states          # Robot joint states
geometry_msgs/PoseStamped flange_pose        # Robot flange pose
geometry_msgs/PoseStamped tool_pose          # Robot tool pose

# alarm code 
AlarmCode[]   alarm_code_list                # Robot alarm code

float32 overall_speed                        # Overall speed (%)


# IO
IOType[] io                                   # TODO: IO status

bool is_error                                 # TODO: implement error detection
uint8 error_code                              # TODO: assign proper error codes on failures


# coordinate system
int32 actived_uf_id                           # Activated user coordinate system ID
int32 actived_tf_id                           # Activated tool coordinate system ID


```

### 机器人软状态

| 功能描述 | 机器人软状态 |
| :---: | :---- |
|接口名称|gbt_interface/msg/ArmSoftModeStatus|
|接口说明|包含以下信息：机器人软模式状态|

**详细内容**

```yaml
int32 status_code  # status code
string status_description  # status description
```

**注意**：机器人操作模式（软模式）在实体机上只能通过手柄钥匙切换，不可以通过页面或者SDK切换。

### 机器人状态

| 功能描述 | 机器人状态 |
| :---: | :---- |
|接口名称|gbt_interface/msg/RobotStatus|
|接口说明|包含以下信息：机器人状态|

**详细内容**

```yaml
int32 status_code       # Status code
string status_description  # Description of the status

```

**状态码说明**

- `ROBOT_IDLE:`机器人空闲,对应状态码0
- `ROBOT_RUNNING:`机器人运行中,对应状态码1
- `ROBOT_TEACHING:` 机器人示教中,对应状态码2
- `ROBOT_IDLE_TO_RUNNING:` 机器人中间状态 空闲转换为运行,对应状态码101
- `ROBOT_IDLE_TO_TEACHING:` 机器人中间状态 空闲转换为示教,对应状态码102
- `ROBOT_RUNNING_TO_IDLE:` 机器人中间状态 运行转换为空闲,对应状态码103
- `ROBOT_TEACHING_TO_IDLE:` 机器人中间状态 示教转换为空闲,对应状态码104
- `ROBOT_UNKNOWN:`机器人未知状态,对应状态码-1

### 控制器状态

| 功能描述 | 控制器状态 |
| :---: | :---- |
|接口名称|gbt_interface/msg/ControllerStatus|
|接口说明|包含以下信息：控制器状态|

**详细内容**

```yaml
int32 status_code  #status code
string status_description  # status description
```

**状态码说明**

- `CTRL_INIT:`控制器初始化,对应状态码0
- `CTRL_ENGAGED:`控制器使能,对应状态码1
- `CTRL_ESTOP:`控制器急停,对应状态码2
- `CTRL_TERMINATED:`控制器中止,对应状态码3
- `CTRL_ANY_TO_ESTOP:`控制器中间状态 其他转换为急停,对应状态码101
- `CTRL_ESTOP_TO_ENGAGED:`控制器中间状态 急停到使能,对应状态码102
- `CTRL_ESTOP_TO_TERMINATED:`控制器中间状态 急停到中止,对应状态码103
- `CTRL_UNKNOWN:`未知的控制器状态,对应状态码-1

### 伺服控制器状态

| 功能描述 | 伺服控制器状态 |
| :---: | :---- |
|接口名称|gbt_interface/msg/ServoStatus|
|接口说明|包含以下信息：伺服控制器状态|

**详细内容**

```yaml
int32 status_code  # status code
string status_description  # status description

```

**状态码说明**  

- `SERVO_IDLE:`伺服控制器空闲,对应状态码1
- `SERVO_RUNNING:`伺服控制器运行中,对应状态码2
- `SERVO_DISABLE:`伺服控制器关闭,对应状态码3
- `SERVO_WAIT_READY:`伺服控制器等待就绪,对应状态码4
- `SERVO_WAIT_DOWN:`伺服控制器等待关闭,对应状态码5
- `SERVO_INIT:`伺服控制器初始化,对应状态码10
- `SERVO_UNKNOWN:`未知的伺服控制器状态,对应状态码-1

## 报警码

| 功能描述 | 报警码信息 |
| :---: | :---- |
|接口名称|gbt_interface/msg/AlarmCode|
|接口说明|包含以下信息：给用户展示的报警码、内部报警码、报警名称、报警原因、建议处理、报警后果、扩展描述|

**详细内容**

```yaml
string user_code            # alarm code for user
string inner_code           # alarm code for inner
string name                 # alarm name
string reason               # alarm reason
string suggest              # suggestion
string consequence          # consequence
string ext_desc             # extended description
```

## IOType

| 功能描述 | IO类型 |
| :---: | :---- |
|接口名称|gbt_interface/msg/IOType|
|接口说明|包含以下信息：IO类型、对应端口列表、对应端口值，端口描述|

**详细内容**

```yaml
# 定义 IO 类型
uint8 SIGNAL_TYPE_DI = 1
uint8 SIGNAL_TYPE_DO = 2
uint8 SIGNAL_TYPE_UI = 3
uint8 SIGNAL_TYPE_UO = 4
uint8 SIGNAL_TYPE_RI = 5
uint8 SIGNAL_TYPE_RO = 6
uint8 SIGNAL_TYPE_GI = 7
uint8 SIGNAL_TYPE_GO = 8

# 当前 IO 的类型
uint8 io_type

# 对应的端口列表
uint16[] ports

# 端口的值
uint8[] values

# 可选：端口描述
string[] descriptions

```

**IO类型说明**

- `SIGNAL_TYPE_DI:`数字输入,对应状态码1
- `SIGNAL_TYPE_DO:`数字输出,对应状态码2
- `SIGNAL_TYPE_UI:`模拟输入,对应状态码3
- `SIGNAL_TYPE_UO:`模拟输出,对应状态码4
- `SIGNAL_TYPE_RI:`继电器输入,对应状态码5
- `SIGNAL_TYPE_RO:`继电器输出,对应状态码6
- `SIGNAL_TYPE_GI:`通用输入,对应状态码7
- `SIGNAL_TYPE_GO:`通用输出,对应状态码8

## 离线轨迹（action）

| 功能描述 | 执行离线轨迹 |
| :---: | :---- |
|接口名称|gbt_interface/action/OfflineTrajectory|
|接口说明|包含以下信息：离线轨迹名称、离线轨迹文件路径、离线轨迹文件内容|

**详细内容**

```yaml
# OfflineTrajectory.action
# request
string trajectory_path #  # path to the trajectory file .csv
# bool use_interpolation=1 #  # whether to use interpolation
---
# response
int32 result
string message
---
sensor_msgs/JointState joint_states #  # joint states of the robot


```

## 移动到点（action）

| 功能描述 | 移动到指定点 |
| :---: | :---- |
|接口名称|gbt_interface/action/MoveToPose|
|接口说明|包含以下信息：目标点位，结果信息，中间位置|

**详细内容**

```yaml
# Request part
float64 x
float64 y
float64 z
float64 a
float64 b
float64 c
float32 vel      # velocity (unit: mm/s)
float32 acc      # acceleration [0,1]

---

# Result part
bool success
string message

---

# Feedback part - Current position
geometry_msgs/Pose current_pose

```

## 急停(service)

| 功能描述 | 机器人急停 |
| :---: | :---- |
|接口名称|gbt_interface/srv/EmergencyStop|

**详细内容**

```yaml
# EmergencyStop.srv
# emergency stop the robot

# Request

---
# Response
bool success        # Success flag
string message      # Message
```

## LED开关  (service)

| 功能描述 | LED开关 |
| :---: | :---- |
|接口名称|gbt_interface/srv/LED|

**详细内容**

```yaml
# Service to control LED on/off

bool led_on  # Target LED state (true = on, false = off)

---

bool success
string message
```

## IO读写（service）

| 功能描述 | IO读写 |
| :---: | :---- |
|接口名称|gbt_interface/srv/IO|

**详细内容**

```yaml
## IO configuration and read service definition

# Request part
uint8 signal_type   # IO signal type
uint8 signal_value  # IO signal value
uint8 signal_port   # IO signal port
string command      # Command, options: "set" or "get"

---

# Response part
bool success        # Success flag
string message      # Result message
uint8 signal_value  # IO signal value (returned value in case of 'get')

```

**IO类型说明**
参考[IO](#io)

## 程序控制（service）

| 功能描述 | 通过程序名控制程序的`start`、`stop` 、`resume`、`stop`|
| :---: | :---- |
|接口名称|gbt_interface/srv/ProgramControl|

**详细内容**

```yaml
# Service for controlling the execution state of a program

# Request part
string command        # Command to execute: "start", "stop", "pause", "resume"
string program_name   # Name of the program

---

# Response part
bool success          # Whether the operation was successful
string message        # Status or error message
```

**命令说明**

- `start:`启动程序
- `stop:`停止程序
- `pause:`暂停程序
- `resume:`恢复程序

## 发送脚本（service）

| 功能描述 | 发送脚本 |
| :---: | :---- |
|接口名称|gbt_interface/srv/SendScript|

**详细内容**

```yaml
# Service for sending a script to the robot

string script_name    # Name of the script to be sent
string script_content # Content of the script

---

bool success
string message        # Result message (e.g., success or error description)
```

## 伺服上下电（service）

| 功能描述 | 伺服上下电 |
| :---: | :---- |
|接口名称|gbt_interface/srv/Servo|

**详细内容**

```yaml

# Service to control servo on/off

bool servo_on  # Target servo state (true = on, false = off)

---

bool success
string message
```
