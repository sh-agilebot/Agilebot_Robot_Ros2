<div align="right">

[中文简体](#)|[English](readme.md)

</div>

[TOC]

<div align="center">

# Shanghai Agilebot Robotic Arm ROS2 Interface Guide - v0.1.0

Revision History:

| Version |        Date       | Notes    |
| :-----: | :---------------: | :------- |
|  v0.0.0 | November 14, 2024 | Drafted  |
|  v0.1.0 |   August 4, 2025  | Released |

</div>

## Introduction

This document describes the ROS2 interface for the Shanghai Agilebot robotic arm.

## Robotic Arm Basic Information

### Robot Basic Information (msg)

| Description | Robot Basic Information                                                                                                                                                                                                  |
| :---------- | :----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Topic Name  | gbt\_interface/msg/FeedbackState                                                                                                                                                                                         |
| Description | Contains the following information: robot type, robot state, controller state, servo controller state, arm soft-mode status, joint states, end-effector pose, alarm codes, overall speed, activated user and tool frames |

**Details**

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

# Alarm codes
AlarmCode[]   alarm_code_list                # Robot alarm codes

float32 overall_speed                        # Overall speed (%)

# IO
IOType[] io                                   # TODO: IO status

bool is_error                                 # TODO: implement error detection
uint8 error_code                              # TODO: assign proper error codes on failures

# Coordinate Frames
int32 actived_uf_id                           # Activated user frame ID
int32 actived_tf_id                           # Activated tool frame ID
```

### Arm Soft Mode Status

| Description | Arm Soft Mode Status                           |
| :---------- | :--------------------------------------------- |
| Topic Name  | gbt\_interface/msg/ArmSoftModeStatus           |
| Description | Contains the soft-mode status of the robot arm |

**Details**

```yaml
int32 status_code          # Status code
string status_description  # Status description
```

**Note**: The robot operation mode (soft mode) on the physical robot can only be switched using the teach pendant key switch. It cannot be switched through the web interface or SDK.

### Robot Status

| Description | Robot Status                     |
| :---------- | :------------------------------- |
| Topic Name  | gbt\_interface/msg/RobotStatus   |
| Description | Contains the status of the robot |

**Details**

```yaml
int32 status_code           # Status code
string status_description   # Description of the status
```

**Status Codes**

* `ROBOT_IDLE`: Robot is idle, code 0
* `ROBOT_RUNNING`: Robot is running, code 1
* `ROBOT_TEACHING`: Robot is in teaching mode, code 2
* `ROBOT_IDLE_TO_RUNNING`: Transition from idle to running, code 101
* `ROBOT_IDLE_TO_TEACHING`: Transition from idle to teaching, code 102
* `ROBOT_RUNNING_TO_IDLE`: Transition from running to idle, code 103
* `ROBOT_TEACHING_TO_IDLE`: Transition from teaching to idle, code 104
* `ROBOT_UNKNOWN`: Unknown state, code -1

### Controller Status

| Description | Controller Status                     |
| :---------- | :------------------------------------ |
| Topic Name  | gbt\_interface/msg/ControllerStatus   |
| Description | Contains the status of the controller |

**Details**

```yaml
int32 status_code          # Status code
string status_description  # Status description
```

**Status Codes**

* `CTRL_INIT`: Controller initializing, code 0
* `CTRL_ENGAGED`: Controller engaged, code 1
* `CTRL_ESTOP`: Emergency stop, code 2
* `CTRL_TERMINATED`: Controller terminated, code 3
* `CTRL_ANY_TO_ESTOP`: Transition from any state to ESTOP, code 101
* `CTRL_ESTOP_TO_ENGAGED`: Transition from ESTOP to engaged, code 102
* `CTRL_ESTOP_TO_TERMINATED`: Transition from ESTOP to terminated, code 103
* `CTRL_UNKNOWN`: Unknown controller state, code -1

### Servo Controller Status

| Description | Servo Controller Status                     |
| :---------- | :------------------------------------------ |
| Topic Name  | gbt\_interface/msg/ServoStatus              |
| Description | Contains the status of the servo controller |

**Details**

```yaml
int32 status_code          # Status code
string status_description  # Status description
```

**Status Codes**

* `SERVO_IDLE`: Servo idle, code 1
* `SERVO_RUNNING`: Servo running, code 2
* `SERVO_DISABLE`: Servo disabled, code 3
* `SERVO_WAIT_READY`: Waiting for servo ready, code 4
* `SERVO_WAIT_DOWN`: Waiting for servo down, code 5
* `SERVO_INIT`: Servo initializing, code 10
* `SERVO_UNKNOWN`: Unknown servo status, code -1

## Alarm Codes

| Description | Alarm Code Information                                                                                                          |
| :---------- | :------------------------------------------------------------------------------------------------------------------------------ |
| Topic Name  | gbt\_interface/msg/AlarmCode                                                                                                    |
| Description | Contains alarm data for user display: user code, internal code, name, reason, suggestion, consequence, and extended description |

**Details**

```yaml
string user_code            # Alarm code for user
string inner_code           # Internal alarm code
string name                 # Alarm name
string reason               # Alarm reason
string suggest              # Suggestion
string consequence          # Consequence
string ext_desc             # Extended description
```

## IOType

| Description | IO Type                                                               |
| :---------- | :-------------------------------------------------------------------- |
| Topic Name  | gbt\_interface/msg/IOType                                             |
| Description | Contains IO type, associated port list, port values, and descriptions |

**Details**

```yaml
# IO Type Definitions
uint8 SIGNAL_TYPE_DI = 1
uint8 SIGNAL_TYPE_DO = 2
uint8 SIGNAL_TYPE_UI = 3
uint8 SIGNAL_TYPE_UO = 4
uint8 SIGNAL_TYPE_RI = 5
uint8 SIGNAL_TYPE_RO = 6
uint8 SIGNAL_TYPE_GI = 7
uint8 SIGNAL_TYPE_GO = 8

# Current IO type
uint8 io_type

# Associated ports
uint16[] ports

# Port values
uint8[] values

# Optional: port descriptions
string[] descriptions
```

**IO Type Codes**

* `SIGNAL_TYPE_DI`: Digital Input, code 1
* `SIGNAL_TYPE_DO`: Digital Output, code 2
* `SIGNAL_TYPE_UI`: Analog Input, code 3
* `SIGNAL_TYPE_UO`: Analog Output, code 4
* `SIGNAL_TYPE_RI`: Relay Input, code 5
* `SIGNAL_TYPE_RO`: Relay Output, code 6
* `SIGNAL_TYPE_GI`: General Input, code 7
* `SIGNAL_TYPE_GO`: General Output, code 8

## Offline Trajectory (action)

| Description | Execute Offline Trajectory                             |
| :---------- | :----------------------------------------------------- |
| Action Name | gbt\_interface/action/OfflineTrajectory                |
| Description | Contains trajectory name, file path, and file contents |

**Details**

```yaml
# OfflineTrajectory.action
# Request\ nstring trajectory_path  # Path to the trajectory .csv file
# bool use_interpolation=1  # Whether interpolation should be used
---
# Response
int32 result
string message
---
sensor_msgs/JointState joint_states  # Joint states of the robot
```

## Move to Pose (action)

| Description | Move to Specified Pose                          |
| :---------- | :---------------------------------------------- |
| Action Name | gbt\_interface/action/MoveToPose                |
| Description | Contains target pose, result info, and feedback |

**Details**

```yaml
# Request part
float64 x
float64 y
float64 z
float64 a
float64 b
float64 c
float32 vel      # Velocity (unit: mm/s)
float32 acc      # Acceleration [0,1]

---

# Result part
bool success
string message

---

# Feedback part - Current pose
geometry_msgs/Pose current_pose
```

## Emergency Stop (service)

| Description  | Robot Emergency Stop             |
| :----------- | :------------------------------- |
| Service Name | gbt\_interface/srv/EmergencyStop |

**Details**

```yaml
# EmergencyStop.srv
# Emergency stop the robot

# Request

---
# Response
bool success        # Success flag
string message      # Message
```

## LED Control (service)

| Description  | LED On/Off             |
| :----------- | :--------------------- |
| Service Name | gbt\_interface/srv/LED |

**Details**

```yaml
# Service to control LED on/off

bool led_on  # Target LED state (true = on, false = off)

---

bool success
string message
```

## IO Read/Write (service)

| Description  | IO Read/Write         |
| :----------- | :-------------------- |
| Service Name | gbt\_interface/srv/IO |

**Details**

```yaml
# IO configuration and read/write service definition

# Request part
uint8 signal_type   # IO signal type
uint8 signal_value  # IO signal value
uint8 signal_port   # IO signal port
string command      # 'set' or 'get'

---

# Response part
bool success        # Success flag
string message      # Result message
uint8 signal_value  # IO signal value (returned when 'get')
```

**IO Type Reference**
Refer to [IOType](#iotype)

## Program Control (service)

| Description  | Control Program Start/Stop/Pause/Resume |
| :----------- | :-------------------------------------- |
| Service Name | gbt\_interface/srv/ProgramControl       |

**Details**

```yaml
# Service for controlling program execution state

# Request part
string command       # Command: 'start', 'stop', 'pause', 'resume'
string program_name  # Program name

---

# Response part
bool success         # Whether successful
string message       # Status or error message
```

**Commands**

* `start`: Start program
* `stop`: Stop program
* `pause`: Pause program
* `resume`: Resume program

## Send Script (service)

| Description  | Send Script                   |
| :----------- | :---------------------------- |
| Service Name | gbt\_interface/srv/SendScript |

**Details**

```yaml
# Service to send a script to the robot

string script_name    # Script name
string script_content # Script content

---

bool success
string message        # Result message
```

## Servo Power (service)

| Description  | Servo Power On/Off       |
| :----------- | :----------------------- |
| Service Name | gbt\_interface/srv/Servo |

**Details**

```yaml
# Service to control servo power

bool servo_on  # Target servo state (true = on, false = off)

---

bool success
string message
```
