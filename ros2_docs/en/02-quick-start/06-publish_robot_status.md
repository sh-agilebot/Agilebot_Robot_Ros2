# Publishing the current state of the robot (Topic)

Publish robot state information to the `/gbt_driver/feedback_states` Topic. Includes connection status of the robotic arm, robot status, controller status, servo status, robotic arm soft mode status, robotic arm joint status, end-effector pose, robot alarm codes, and speed percentage. Subscribe to this topic to obtain real-time status information of the robotic arm.

## Supported Robot Connections

This driver supports three types of robot connections:

1. **Real Robot** - Physical robot connected via network IP (default: `10.27.1.254`)
2. **Virtual Controller** - Simulated robot for testing (uses same default IP as real robot)
3. **AirBot Cloud Robot** - Remote robot accessed through AirBot cloud service (requires registration)

## Message Structure 
`gbt_interface/msg/FeedbackState.msg`

The following is the message structure for robotic arm status information:

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

## Known Limitation (IO)

- The `io` field in `/gbt_driver/feedback_states` is not published by default.
- The SDK currently supports batch reads for `DO` only; other IO types do not support batch read.
- For IO read access, use per-port service calls via `/gbt_driver/service_server/io`.

## User Guide

### Configure Robot Connection

Modify the robot connection in the `gbt_driver/config/robot_config.yaml` configuration file.

#### For Real Robot / Virtual Controller

Both real robot and virtual controller use the same default IP address:

```yaml
robot_ip_address: "10.27.1.254"  # Default IP for real robot and virtual controller
```

The system will automatically connect to the virtual controller if the real robot is unavailable at this IP.

#### For AirBot Cloud Robot

Access remote robots through the AirBot cloud service:

1. Visit http://airbot.sh-agilebot.com/ to register and apply for a cloud robot
2. After registration, you will receive an assigned robot IP address
3. Configure your assigned cloud robot address:

```yaml
robot_ip_address: "YOUR_ASSIGNED_IP"  # Replace with your assigned AirBot cloud robot IP
```

### Start ROS2 Driver Node

Launch the `gbt_driver` node in the terminal:

```bash
source install/setup.bash
ros2 launch gbt_driver gbt_feedback.launch.py
```

### Obtain Robotic Arm Status Information

After starting the `gbt_driver` node, the driver will automatically connect to the robotic arm and begin publishing status information to the following Topic:

- `/gbt_driver/feedback_states`

By subscribing to this topic, you can obtain the following information:

- Robotic arm connection status
- Robot status
- Controller status
- Servo status
- Robotic arm soft mode status
- Robotic arm joint status
- End-effector pose
- Robot alarm codes
- Speed percentage
- UF coordinate system ID
- TF coordinate system ID

For example, you can use the following command to view the robotic arm's status information:

```bash
source install/setup.bash
ros2 topic echo /gbt_driver/feedback_states
```
