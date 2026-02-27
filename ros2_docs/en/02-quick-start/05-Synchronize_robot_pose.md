# Synchronize Robot Pose to RViz Display

 When using MoveIt2 or Gazebo for path planning and simulation, accessing the **real-time pose** of the physical robot is often required as a reference. This functionality **collects** the robot's joint state data in real time and publishes it to the `/joint_states` topic, enabling synchronization of the robot's true pose. Data published to this topic can be **subscribed to and used by** tools like MoveIt2, Gazebo, and RViz for **displaying** the current robot state, **planning** paths, or **driving** simulations.

## Prerequisites

Before proceeding, ensure:

1. You have completed [Environment Installation and Project Build](../01-installation/index.md)
2. The Agilebot Robot SDK is installed (see [Build Project](../01-installation/02-build.md))
3. The robot controller is powered on and connected to the network

## Configure Robot IP Address

Edit the `gbt_driver/config/robot_config.yaml` file and set the `robot_ip_address` field to the actual IP address of your robot:

```bash
vi gbt_driver/config/robot_config.yaml
```

```yaml
robot_ip_address: "192.168.x.x"  # Replace with actual robot IP
```

## Start Robot State Synchronization

Launch RViz with robot state synchronization using the following command:

```bash
source install/setup.bash
ros2 launch gbt_driver gbt_bridge.launch.py robot_type:=<robot_type>
```

### Supported Robot Models

- `C5A` - 5kg payload collaborative robot
- `C7A` - 7kg payload collaborative robot
- `C12A` - 12kg payload collaborative robot
- `C16A` - 16kg payload collaborative robot

### Example

For C5A model:

```bash
source install/setup.bash
ros2 launch gbt_driver gbt_bridge.launch.py robot_type:=C5A
```

Other robot models can be launched similarly:

```bash
# C7A robot
ros2 launch gbt_driver gbt_bridge.launch.py robot_type:=C7A

# C12A robot
ros2 launch gbt_driver gbt_bridge.launch.py robot_type:=C12A

# C16A robot
ros2 launch gbt_driver gbt_bridge.launch.py robot_type:=C16A
```

## Verify Synchronization

### Method 1: Check RViz Display

After launching, the RViz window will automatically open. You should see:
- The robot's URDF model
- Real-time updating joint states

### Method 2: Check Topic Data

In another terminal, view the `/joint_states` topic:

```bash
source install/setup.bash
ros2 topic echo /joint_states
```

You should see output similar to:

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

### Method 3: Check Detailed Feedback Status (Optional)

For more detailed robot status information (including alarm codes, servo status, etc.), you can additionally start the `robot_status` node:

```bash
source install/setup.bash
ros2 launch gbt_driver gbt_feedback.launch.py
```

Then view the `/gbt_driver/feedback_states` topic:

```bash
ros2 topic echo /gbt_driver/feedback_states
```

## Troubleshooting

### Cannot Connect to Robot

1. Verify the robot IP address is correctly configured
2. Check network connectivity: `ping <robot IP>`
3. Ensure the robot controller is powered on

### Robot Pose Not Updating in RViz

1. Check if `/joint_states` topic is being published:
   ```bash
   ros2 topic hz /joint_states
   ```
2. Ensure the robot is in an operable state (not in alarm state)

### Module Import Error

If you encounter `ModuleNotFoundError: No module named 'Agilebot'`, ensure the SDK is installed:

```bash
pip install Agilebot.Robot.SDK.A-*.whl
```

---

> **Note:**
> - Replace `<robot_type>` with the actual robot model name (e.g., `C5A`, `C7A`, `C12A`, `C16A`).
> - Ensure the robot's network configuration is correct before launching.
> - For additional configuration details, refer to the [Agilebot ROS2 github](https://github.com/sh-agilebot/Agilebot_Robot_Ros2).