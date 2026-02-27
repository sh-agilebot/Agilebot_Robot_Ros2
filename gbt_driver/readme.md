<div align="right">

[中文简体](readme_cn.md)|[English](readme.md)

</div>

[TOC]

<div align="center">

# Shanghai Agilebot Robotic Arm ROS2 Driver User Manual (ROS2) - v0.1.0

## Document Revision History

| Version | Date         | Remarks     |
| :-----: | :----------: | :---------- |
| N/A | Nov 14, 2024 | Initial Version |
| v0.1.0 | August 4, 2025 |Release |

</div>

---

## 1. Introduction

This document describes the usage of the ROS2 driver for Shanghai Agilebot robotic arms. This driver enables:

- Publishing robotic arm status information
- Controlling robotic arm motion
- Synchronizing physical robot state to URDF
- Service Server: Provides interfaces for motion control, IO control, program control, and servo power management
- Action Server: MoveIt2 control for physical robotic arms
- receive trajectory file and execute it on the physical robotic arm

---

## 2. Core Functionality

### 2.1 Publish Robot Status Information

#### Message Structure

`gbt_interface/msg/FeedbackState.msg`

The status message structure is as follows:

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
# Note: The robot operation mode (soft mode) on the physical robot can only be switched using the teach pendant key switch. It cannot be switched through the web interface or SDK.

# Robot joint 
sensor_msgs/JointState joint_states          # Robot joint states
geometry_msgs/PoseStamped flange_pose        # Robot flange pose
geometry_msgs/PoseStamped tool_pose          # Robot tool pose

# Alarm code 
AlarmCode[] alarm_code_list                  # Robot alarm codes

float32 speed_percentage                     # Speed percentage
```

#### Configure Robot IP Address

Modify the robot IP address in `gbt_driver/config/robot_config.yaml`:

```yaml
robot_ip_address: "192.168.x.x"  # Replace with actual robot IP
```

#### Launch ROS2 Driver Node

Start the `gbt_driver` node:

```bash
source install/setup.bash
ros2 launch gbt_driver gbt_feedback.launch.py
```

#### Retrieve Robot Status

After launching, the driver will automatically connect and publish status to:

- `/gbt_driver/feedback_states`

To view status data:

```bash
source install/setup.bash
ros2 topic echo /gbt_driver/feedback_states
```

### 2.2 Synchronize Physical Robot State to URDF

#### Launch RVIZ2

```bash
source install/setup.bash
ros2 launch gbt_driver gbt_bridge.launch.py
```

> Robot state will be visualized in RVIZ.

### 2.3 Service Server

#### Move to Pose

| Description | Move to specified pose |
| :---------: | :--------------------- |
| Interface | `gbt_interface/action/MoveToPose` |
| Function | Moves robot to target pose in Cartesian space |
| Launch | `ros2 launch gbt_driver gbt_service_server.launch.py` |
| Example | `ros2 action send_goal /gbt_driver/move_to_pose gbt_interface/action/MoveToPose "{ x: 400.0, y: -200.0, z: 500.0, a: 100.0, b: 30.0, c: 110, vel: 1, acc: 1 }"` |
| Notes | Uses Cartesian coordinates. `vel` and `acc` are multipliers (0-1). |

#### IO Control

**Set IO**

| Description | Set IO state |
| :---------: | :----------- |
| Interface | `gbt_interface/srv/IO` |
| Function | Controls digital/analog IO ports |
| Launch | `ros2 launch gbt_driver gbt_service_server.launch.py` |
| Example | `ros2 service call /gbt_driver/service_server/io gbt_interface/srv/IO "{signal_type: 2, signal_port: 1, signal_value: 1, command: set}"` |
| signal_type | Options: `DI=1`, `DO=2`, `UI=3`, `UO=4`, `RI=5`, `RO=6`, `GI=7`, `GO=8` |

**Get IO**

| Description | Get IO state |
| :---------: | :----------- |
| Interface | `gbt_interface/srv/IO` |
| Function | Reads digital/analog IO ports |
| Launch | `ros2 launch gbt_driver gbt_service_server.launch.py` |
| Example | `ros2 service call /gbt_driver/service_server/io gbt_interface/srv/IO "{signal_type: 1, signal_port: 1, command: get}"` |

#### Program Control

| Description | Control program execution |
| :---------: | :------------------------ |
| Interface | `gbt_interface/srv/ProgramControl` |
| Function | Start/pause/resume/stop programs |
| Launch | `ros2 launch gbt_driver gbt_service_server.launch.py` |
| Example | `ros2 service call /gbt_driver/service_server/program_control gbt_interface/srv/ProgramControl "{command: 'start', program_name: 'test'}"` |
| Command | Options: `start`, `pause`, `resume`, `stop` |

#### Servo Power Control

| Description | Servo power management |
| :---------: | :--------------------- |
| Interface | `gbt_interface/srv/Servo` |
| Function | Enable/disable servo power |
| Launch | `ros2 launch gbt_driver gbt_service_server.launch.py` |
| Example | `ros2 service call /gbt_driver/service_server/servo_power gbt_interface/srv/Servo "{servo_on: true}"` |

#### Send Script

| Description | Execute custom script |
| :---------: | :------------------- |
| Interface | `gbt_interface/srv/SendScript` |
| Function | Upload and execute robot script |
| Launch | `ros2 launch gbt_driver gbt_service_server.launch.py` |
| Example | `ros2 service call /gbt_driver/service_server/send_script gbt_interface/srv/SendScript "{script_name: 'test', script_content: 'SUB main\n MOVEJ PR[1] 100 SD 200.5\n RETURN\nEND'}"` |

### 2.5 Action Server (Experimental)

**Description:** Translate the trajectory computed by MoveIt into an offline trajectory for the robotic arm, and drive the arm to execute it.

**Note**: Please operate in automatic mode (The robot operation mode on the physical robot can only be switched using the teach pendant key switch. It cannot be switched through the web interface or SDK).

**Configuration:**  
Set robot IP in `gbt_driver/config/robot_config.yaml`.

**Terminal 1:** Launch Action Server  

```bash
source install/setup.bash
ros2 launch gbt_driver gbt_action_server.launch.py <robot_type>:= <robot_type> <interpolate_mode>:=<quintic or spline>

```

> Options: quintic or spline, default is quintic.
> quintic is a quintic polynomial interpolation method, and spline is a quintic B-spline interpolation method.

EXample for C5A model:

```bash
ros2 launch gbt_driver gbt_action_server.launch.py robot_type:=C5A interpolate_mode:=quintic
```

### 2.6 Offline Trajectory  

**Description:** Control robot arm movement through an offline trajectory file (CSV) to achieve complex and precise motions. For reference on trajectory file format, see: [gbt_driver/gbt_driver/test.csv](gbt_driver/test.csv)  

#### Trajectory File Specification  

Offline trajectory files are in CSV format with the following header:  

```csv  
ts,pts_J1,pts_J2,pts_J3,pts_J4,pts_J5,pts_J6,vel_J1,vel_J2,vel_J3,vel_J4,vel_J5,vel_J6,acc_J1,acc_J2,acc_J3,acc_J4,acc_J5,acc_J6,jerk_J1,jerk_J2,jerk_J3,jerk_J4,jerk_J5,jerk_J6,do_port,do_state  
```  

##### Format Details  

**1. `ts` (Timestamp)**  

- **Meaning**: Represents timing information for the current trajectory point, typically in seconds (s).  
- **Purpose**: Synchronizes the robot arm's movement timing to ensure joints reach target positions, velocities, and accelerations at specified time points.  

**2. `pts_J1` to `pts_J6` (Joint Positions)**  

- **Meaning**:  
  - `pts_J1` to `pts_J6` represent the **positions of the 6 joints** of the robotic arm (Joint Position).  
  - Units: Radians (rad).  
- **Purpose**:  
  - Defines the spatial position and orientation of the end-effector (via forward kinematics).  
  - Core data for trajectory planning, describing the path from start to end points.  

**3. `vel_J1` to `vel_J6` (Joint Velocities)**  

- **Meaning**:  
  - `vel_J1` to `vel_J6` represent the **velocities of the 6 joints** (Joint Velocity).  
  - Units: Radians per second (rad/s).  
- **Purpose**:  
  - Controls joint movement speeds to ensure smooth end-effector motion along the planned path.  
  - Prevents mechanical shocks or vibrations caused by abrupt speed changes.  

**4. `acc_J1` to `acc_J6` (Joint Accelerations)**  

- **Meaning**:  
  - `acc_J1` to `acc_J6` represent the **accelerations of the 6 joints** (Joint Acceleration).  
  - Units: Radians per second squared (rad/s²).  
- **Purpose**:  
  - Describes the rate of change of joint velocity, affecting the robot's dynamic performance.  
  - Excessive acceleration may cause overloading or unwanted vibrations.  

**5. `jerk_J1` to `jerk_J6` (Joint Jerks)**  

- **Meaning**:  
  - `jerk_J1` to `jerk_J6` represent the **jerks of the 6 joints** (Joint Jerk).  
  - Units: Radians per second cubed (rad/s³).  
- **Purpose**:  
  - Describes the rate of change of acceleration, directly impacting motion smoothness.  

**6. `do_port` (Digital Output Port)**  

- **Meaning**:  
  - Specifies the **digital output port number** (Digital Output Port) of the robot arm.  
  - Used to control external devices (e.g., grippers, vacuum cups, sensors).  
- **Purpose**:  
  - Triggers on/off states of external devices at specific time points (e.g., gripper activation).  

**7. `do_state` (Digital Output State)**  

- **Meaning**:  
  - Represents the **state** of the digital output port (`do_port`), typically a boolean value (0 or 1).  
  - Example:  
    - `0`: Off (e.g., gripper open).  
    - `1`: On (e.g., gripper closed).  
- **Purpose**:  
  - Coordinates robot motion with external device actions (e.g., triggering a gripper after reaching a target position).  

#### Usage Instructions  

**Set Robot IP Address**  
In `gbt_driver/config/robot_config.yaml`, set `robot_ip_address` to the robot's IP address.  

##### Launch Service  

```bash  
source install/setup.bash  
ros2 launch gbt_offline_trajectory.launch.py  
```  

##### Interface Example  

[sample.csv](../assets/sample.csv)

```bash
ros2 action send_goal gbt_driver/trajectory gbt_interface/action/OfflineTrajectory "{trajectory_path: '/home/gbt/tmp/src/Agilebot_Robot_Ros2/assets/sample.csv'}"
```

### 2.7 Emergency Stop

|     Description     | Emergency Stop                                                                                     |
| :-----------------: | :------------------------------------------------------------------------------------------------- |
|     Service Name    | `gbt_interface/srv/EmergencyStop`                                                                  |
| Service Description | This service is used to control the emergency stop state of the robot arm.                         |
|    Launch Command   | `ros2 launch gbt_driver gbt_service_server.launch.py`                                              |
|    Usage Example    | `ros2 service call /gbt_driver/service_server/emergency_stop gbt_interface/srv/EmergencyStop "{}"` |

### 2.8 LED Switch Control

|       Description      | LED switch control                                                                        |
| :--------------------: | :---------------------------------------------------------------------------------------- |
|      Service Name      | `gbt_interface/srv/LED`                                                                   |
| Description of Service | This service is used to control the on/off state of the robot arm’s LED lights.           |
|     Launch Service     | `ros2 launch gbt_driver gbt_service_server.launch.py`                                     |
|      Usage Example     | `ros2 service call /gbt_driver/service_server/led gbt_interface/srv/LED "{led_on: true}"` |

---

## 3. Important Notes

- Ensure correct robot IP configuration.
- ROS2 and robot must be on the same network.
- Check firewall settings if connection issues occur.

---

## 4. License

This project is licensed under BSD 3-Clause License.
