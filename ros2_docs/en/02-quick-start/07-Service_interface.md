# Common Robot Control Interfaces

Provides commonly used robot control interfaces, including moving to specified positions, IO control, program execution control, and servo power on/off control,Emergency stop.

> **Note:**  
> - `robot_type` defaults to `"C5A"`.  
> - Optional values: `C7A`, `C12A`, `C16A`.  
> - `move_to_pose`, `program_control`, and `send_script` require a runnable robot state (`CTRL_ENGAGED` with no active alarms).  
> - If `emergency_stop` was triggered, clear/reset alarms first (teach pendant or SDK) before motion/script commands.  

## Robot End-effector Movement to Target Position

### Interface Name

- `gbt_interface/action/MoveToPose`

### Usage Method

Start service:

```bash
ros2 launch gbt_driver gbt_service_server.launch.py robot_type:=<robot_type>
```
For headless environments (Docker/CI), disable RViz to avoid display errors:
```bash
ros2 launch gbt_driver gbt_service_server.launch.py robot_type:=<robot_type> enable_rviz:=false
```

Simulate sending target position request:

```bash
ros2 action send_goal /gbt_driver/move_to_pose gbt_interface/action/MoveToPose "{ x: 400.0, y: -200.0, z: 500.0, a: 100.0, b: 30.0, c: 110, vel: 1, acc: 1 }"
```

## IO Read/Write
> **Note**:  Please configure the I/O mapping before using the I/O control function.

- **Interface Name**: `gbt_interface/srv/IO`
- **Start Service**:

```bash
ros2 launch gbt_driver gbt_service_server.launch.py robot_type:=<robot_type>
```

- **Set IO Port Status**:

```bash
ros2 service call /gbt_driver/service_server/io gbt_interface/srv/IO "{signal_type: 2, signal_port: 1, signal_value: 1, command: 'set'}"
```

- **Get IO Port Status**:

```bash
ros2 service call /gbt_driver/service_server/io gbt_interface/srv/IO "{signal_type: 2, signal_port: 1, command: 'get'}"
```

### IO TYPE Table
| Enum | Value | Description          |
| ---- | ----- | -------------------- |
| DI   | 1     | Digital Input        |
| DO   | 2     | Digital Output       |
| UI   | 3     | Universal Input      |
| UO   | 4     | Universal Output     |
| RI   | 5     | Robot Input          |
| RO   | 6     | Robot Output         |
| GI   | 7     | Group Input          |
| GO   | 8     | Group Output         |
| TAI  | 9     | Wrist Analog Input   |
| TDI  | 10    | Wrist Digital Input  |
| TDO  | 11    | Wrist Digital Output |
| AI   | 12    | Analog Input         |
| AO   | 13    | Analog Output        |


## Program Execution Control

- **Interface Name**: `gbt_interface/srv/ProgramControl`
- **Start Service**:

```bash
ros2 launch gbt_driver gbt_service_server.launch.py robot_type:=<robot_type>
```

- **Start Program**:

```bash
ros2 service call /gbt_driver/service_server/program_control gbt_interface/srv/ProgramControl "{command: 'start', program_name: 'test'}"
```

To pause, resume or stop programs, replace the `command` parameter with:
- `pause`: Pause program
- `resume`: Resume program
- `stop`: Stop program

If controller state is `CTRL_ESTOP` or there are active alarms, this interface may return `CONTROLLER_INVALID_OPERATION_START`.

## Servo Power Control

- **Interface Name**: `gbt_interface/srv/Servo`
- **Start Service**:

```bash
ros2 launch gbt_driver gbt_service_server.launch.py robot_type:=<robot_type>
```

- **Servo Motor Power On**:

```bash
ros2 service call /gbt_driver/service_server/servo_power gbt_interface/srv/Servo "{servo_on: true}"
```

To power off the servo motor, set `servo_on` parameter to `false`.


## Emergency Stop
- **Interface Name**: `gbt_interface/srv/EmergencyStop`
- **Interface Description**: Controls the emergency stop state of the robotic arm
- **Start Service**:
  ```bash
  ros2 launch gbt_driver gbt_service_server.launch.py robot_type:=<robot_type>
  ```
- **Usage Example**:
  ```bash
  ros2 service call /gbt_driver/service_server/emergency_stop gbt_interface/srv/EmergencyStop "{}"
  ```
> Note: After emergency stop, the controller may stay in E-Stop state. Reset alarms before motion/script commands.
> If the controller is already in `CTRL_ESTOP`, this call may return `CONTROLLER_INVALID_OPERATION`.


## LED Switch
- **Interface Name**: `gbt_interface/srv/LED`
- **Interface Description**: This interface is used to control the LED status of the robotic arm
- **Start Service**:
  ```bash
  ros2 launch gbt_driver gbt_service_server.launch.py robot_type:=<robot_type>
  ```
- **Usage Example**:
  ```bash
  ros2 service call /gbt_driver/service_server/led gbt_interface/srv/LED "{led_on: true}"
  ```
