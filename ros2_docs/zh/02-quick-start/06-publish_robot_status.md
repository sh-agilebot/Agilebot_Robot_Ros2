# 发布机器人当前状态（Topic）

发布机器人状态信息，到`/gbt_driver/feedback_states` Topic。包括机械臂的连接状态、机器人状态、控制器状态、伺服状态、机械臂软模式状态、机械臂的关节状态、末端执行器的位姿、机器人报警码和速度百分比。订阅这个话题，您可以获取机械臂的实时状态信息。

## 支持的机器人连接类型

本驱动支持三种类型的机器人连接：

1. **真实机器人** - 通过网络IP连接的物理机器人（默认：`10.27.1.254`）
2. **虚拟控制器** - 用于测试的模拟机器人（使用与真实机器人相同的默认IP）
3. **AirBot云机器人** - 通过AirBot云服务访问的远程机器人（需要注册）

## 消息结构
`gbt_interface/msg/FeedbackState.msg`

以下是机械臂状态信息的消息结构：

```yaml
std_msgs/Header header

# Connection Status
bool is_connected                            # Whether connected to the robot

# Robot Status
string robot_type                            # Robot type
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
IOType[] io                                  # TODO: IO status

bool is_error                                # TODO: implement error detection
uint8 error_code                             # TODO: assign proper error codes on failures


# coordinate system
int32 actived_uf_id                          # Activated user coordinate system ID
int32 actived_tf_id                          # Activated tool coordinate system ID


```

## 已知限制（IO）

- 当前 `/gbt_driver/feedback_states` 的 `io` 字段默认未发布。
- 原因是 SDK 目前仅支持 `DO` 批量读取，其他 IO 类型不支持批量读取。
- 如需读取 IO，建议通过服务接口逐端口读取：`/gbt_driver/service_server/io`。

## 使用指南

### 配置机器人连接

在 `gbt_driver/config/robot_config.yaml` 配置文件中，修改机器人连接配置。

#### 真实机器人 / 虚拟控制器

真实机器人和虚拟控制器使用相同的默认IP地址：

```yaml
robot_ip_address: "10.27.1.254"  # 真实机器人和虚拟控制器的默认IP
```

如果该IP地址上没有真实机器人，系统将自动连接到虚拟控制器。

#### AirBot云机器人

通过AirBot云服务访问远程机器人：

1. 访问 http://airbot.sh-agilebot.com/ 注册并申请云机器人
2. 注册后，您将收到分配的机器人IP地址
3. 配置您分配的云机器人地址：

```yaml
robot_ip_address: "YOUR_ASSIGNED_IP"  # 替换为您分配的AirBot云机器人IP
```

### 启动 ROS2 驱动节点

在终端启动 `gbt_driver` 节点：

```bash
source install/setup.bash
ros2 launch gbt_driver gbt_feedback.launch.py
```

### 获取机械臂状态信息

启动 `gbt_driver` 节点后，驱动会自动连接到机械臂，并开始发布机械臂的状态信息到以下Topic：

- `/gbt_driver/feedback_states`

通过订阅该话题，您可以获取以下信息：

- 机械臂连接状态
- 机器人状态
- 控制器状态
- 伺服状态
- 机械臂软模式状态
- 机械臂的关节状态
- 末端执行器的位姿
- 机器人报警码
- 速度百分比
- UF和 TF 坐标系ID

例如，您可以使用以下命令来查看机械臂的状态信息：

```bash
source install/setup.bash
ros2 topic echo /gbt_driver/feedback_states
```
