# 控制机器人的常用接口

提供常见的控制机器人接口，包括移动到指定位置、IO控制、控制程序运行、伺服上下电控制、紧急停止。

> **注意：**  
> - `robot_type` 默认值为 `"C5A"`。  
> - 可选值：`C7A`、`C12A`、`C16A`。  
> - `move_to_pose`、`program_control`、`send_script` 需要机器人处于可执行状态（控制器 `CTRL_ENGAGED`，且无活动报警）。  
> - 若执行过 `emergency_stop`，需要先在示教器或通过 SDK 复位报警后再执行运动/脚本相关指令。  
> 
## 机器人末端移动到指定位置

### 接口名称

- `gbt_interface/action/MoveToPose`

### 使用方法

启动服务：

```bash
ros2 launch gbt_driver gbt_service_server.launch.py robot_type:=<机器人型号>
```
如果在无图形界面环境（Docker/CI）运行，建议关闭 RViz 避免显示报错：
```bash
ros2 launch gbt_driver gbt_service_server.launch.py robot_type:=<机器人型号> enable_rviz:=false
```

模拟发送目标位置请求：

```bash
ros2 action send_goal /gbt_driver/move_to_pose gbt_interface/action/MoveToPose "{ x: 400.0, y: -200.0, z: 500.0, a: 100.0, b: 30.0, c: 110, vel: 1, acc: 1 }"
```



## IO读写

>**注意**: 使用该功能前，请确保已经设置I/O映射

- **接口名称**: `gbt_interface/srv/IO`
- **启动服务**:

```bash
ros2 launch gbt_driver gbt_service_server.launch.py robot_type:=<机器人型号>
```

- **设置IO端口状态**:

```bash
ros2 service call /gbt_driver/service_server/io gbt_interface/srv/IO "{signal_type: 2, signal_port: 1, signal_value: 1, command: 'set'}"
```

- **获取IO端口状态**:

```bash
ros2 service call /gbt_driver/service_server/io gbt_interface/srv/IO "{signal_type: 2, signal_port: 1, command: 'get'}"
```

### IO信号类型表
| 枚举  | 值  | 描述     |
| --- | -- | ------ |
| DI  | 1  | 数字输入   |
| DO  | 2  | 数字输出   |
| UI  | 3  | 通用输入   |
| UO  | 4  | 通用输出   |
| RI  | 5  | 机器人输入   |
| RO  | 6  | 机器人输出   |
| GI  | 7  | 组输入    |
| GO  | 8  | 组输出    |
| TAI | 9  | 手腕模拟输入 |
| TDI | 10 | 手腕数字输入 |
| TDO | 11 | 手腕数字输出 |
| AI  | 12 | 模拟输入   |
| AO  | 13 | 模拟输出   |



## 控制程序运行
>**注意**: 使用该功能前，请确保存在该程序

- **接口名称**: `gbt_interface/srv/ProgramControl`
- **启动服务**:

```bash
ros2 launch gbt_driver gbt_service_server.launch.py robot_type:=<机器人型号>
```

- **启动程序**:


```bash
ros2 service call /gbt_driver/service_server/program_control gbt_interface/srv/ProgramControl "{command: 'start', program_name: 'test'}"
```

如果需要暂停、恢复或者停止程序，请将`command`参数替换为下面选项： 
- `pause`： 暂停程序, 
- `resume`: 恢复程序,
- `stop`: 停止程序,

若控制器状态为 `CTRL_ESTOP` 或存在活动报警，该接口可能返回 `CONTROLLER_INVALID_OPERATION_START`。



## 伺服上下电控制

- **接口名称**: `gbt_interface/srv/Servo`
- **启动服务**:

```bash
ros2 launch gbt_driver gbt_service_server.launch.py robot_type:=<机器人型号>
```

- **控制伺服电机上电**:

```bash
ros2 service call /gbt_driver/service_server/servo_power gbt_interface/srv/Servo "{servo_on: true}"
```

如果需要控制伺服电机下电，请将`servo_on`参数替换为`false`。

## 紧急停止
- **接口名称**: `gbt_interface/srv/EmergencyStop`
- **接口说明**: 该接口用于控制机械臂的紧急停止状态
- **启动服务**:
  ```bash
  ros2 launch gbt_driver gbt_service_server.launch.py robot_type:=<机器人型号>
  ```
- **接口使用示例**:
  ```bash
  ros2 service call /gbt_driver/service_server/emergency_stop gbt_interface/srv/EmergencyStop "{}"
  ```
> 注意：急停后控制器会进入急停状态，后续运动/脚本相关接口可能返回状态异常，需要先复位。
> 若控制器已处于 `CTRL_ESTOP`，该接口可能返回 `CONTROLLER_INVALID_OPERATION`。
## LED开关
- **接口名称**: `gbt_interface/srv/LED`
- **接口说明**: 该接口用于控制机械臂的LED灯状态
- **启动服务**:
  ```bash
  ros2 launch gbt_driver gbt_service_server.launch.py robot_type:=<机器人型号>
  ```
- **接口使用示例**:
  ```bash
  ros2 service call /gbt_driver/service_server/led gbt_interface/srv/LED "{led_on: true}"
  ```




