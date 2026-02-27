# Control Robot Motion Using Offline Trajectory Files  
Receive an offline trajectory file that conforms to the Agilebot robot specification, send it to the robot via the robot interface, and the robot will execute the commands in the trajectory file. This enables finer-grained trajectory control—including position, velocity, acceleration, and jerk parameters—as well as digital output port control. Users may choose their preferred trajectory-planning method (for example, using MoveIt 2 or another planner), save the resulting trajectory as a CSV file, and then send it to the robot through this interface.


> **Note:**
>
> 1. This feature requires the robot to be set to `AUTO` mode. Please ensure the robot is in this mode; otherwise, execution will fail. You can switch to `AUTO` by turning the key switch on the pendant to the `AUTO` position.
>
> 2. Please ensure the robot software version is >= 7.6.7.0; otherwise, this feature will not be available.


## Preparing Offline Trajectory Files  
Offline trajectory files are in CSV format with the following header:  
```csv
ts,pts_J1,pts_J2,pts_J3,pts_J4,pts_J5,pts_J6,vel_J1,vel_J2,vel_J3,vel_J4,vel_J5,vel_J6,acc_J1,acc_J2,acc_J3,acc_J4,acc_J5,acc_J6,jerk_J1,jerk_J2,jerk_J3,jerk_J4,jerk_J5,jerk_J6,do_port,do_state
```

Example file:[sample.csv](/assets/sample.csv)
```csv
ts,pts_J1,pts_J2,pts_J3,pts_J4,pts_J5,pts_J6,vel_J1,vel_J2,vel_J3,vel_J4,vel_J5,vel_J6,acc_J1,acc_J2,acc_J3,acc_J4,acc_J5,acc_J6,jerk_J1,jerk_J2,jerk_J3,jerk_J4,jerk_J5,jerk_J6,do_port,do_state
0,0.0000873,0.00020944,-0.350217768,-0.001047198,0.348018653,0,0,0,0,0,0,0,2.843203747,3.305122443,-0.822163233,12.12931686,-17.22206528,-16.64150606,0,0,0,0,0,0,-1,0
0.001000026,0.0000887,0.000211091,-0.350218178,-0.001041137,0.348010048,-0.00000831,0.00283975,0.003301588,-0.000821221,0.012115944,-0.017202583,-0.016622901,2.836146722,3.297879454,-0.820235299,12.10193265,-17.18219516,-16.60342136,-7.057025559,-7.24298847,1.927934515,-27.3842142,39.87011938,38.08470461,-1,0
0.002000053,0.0000929,0.00021604,-0.35021941,-0.001022974,0.34798426,-0.0000332,0.005672443,0.006595933,-0.001640514,0.024204502,-0.034365295,-0.033207717,2.829089696,3.290636466,-0.818307364,12.07454844,-17.14232504,-16.56533665,-7.057025559,-7.24298847,1.927934515,-27.3842142,39.87011938,38.08470461,-1,0
0.003000079,0.00010003,0.000224281,-0.350221459,-0.000992736,0.347941329,-0.0000747,0.008498078,0.009883034,-0.002457879,0.036265676,-0.051488136,-0.049754447,2.822032671,3.283393477,-0.816379429,12.04716422,-17.10245492,-16.52725195,-7.057025559,-7.24298847,1.927934515,-27.3842142,39.87011938,38.08470461,-1,0

```

## Trajectory File Format Description  
**1. `ts` (Timestamp)**  
- **Meaning**: Represents time information for the current trajectory point, typically in seconds (s).  
- **Purpose**: Synchronizes robotic arm motion timing to ensure joints reach target positions, velocities, and accelerations at specified time points.  

**2. `pts_J1` to `pts_J6` (Joint Positions)**  
- **Meaning**:  
  - `pts_J1` to `pts_J6` represent the **6 joint positions (Joint Position)** of the robotic arm.  
  - Units: Radians (rad).  
- **Purpose**:  
  - Defines the spatial position and orientation of the end-effector via forward kinematics.  
  - Core trajectory planning data describing the arm's path from start to end points.  

**3. `vel_J1` to `vel_J6` (Joint Velocities)**  
- **Meaning**:  
  - `vel_J1` to `vel_J6` represent the **6 joint velocities (Joint Velocity)**.  
  - Units: Radians per second (rad/s).  
- **Purpose**:  
  - Controls joint motion speeds to ensure smooth end-effector movement along planned paths.  
  - Prevents mechanical shocks or vibrations caused by sudden velocity changes.  

**4. `acc_J1` to `acc_J6` (Joint Accelerations)**  
- **Meaning**:  
  - `acc_J1` to `acc_J6` represent the **6 joint accelerations (Joint Acceleration)**.  
  - Units: Radians per second squared (rad/s²).  
- **Purpose**:  
  - Describes joint acceleration rates affecting dynamic performance.  
  - Excessive acceleration may cause overloading or unwanted vibrations.  

**5. `jerk_J1` to `jerk_J6` (Joint Jerks)**  
- **Meaning**:  
  - `jerk_J1` to `jerk_J6` represent the **6 joint jerks (Joint Jerk)**.  
  - Units: Radians per second cubed (rad/s³).  
- **Purpose**:  
  - Describes acceleration rate-of-change, directly impacting motion smoothness.  

**6. `do_port` (Digital Output Port)**  
- **Meaning**:  
  - Represents the **digital output port number (Digital Output Port)** of the robotic arm.  
  - Used to control external devices (e.g., grippers, vacuum cups, sensors).  
- **Purpose**:  
  - Triggers on/off states of external devices at specific times (e.g., gripper grasp/release).  

**7. `do_state` (Digital Output State)**  
- **Meaning**:  
  - Represents the **state (State)** of the digital output port (`do_port`), typically a boolean (0 or 1).  
  - Example:  
    - `0`: Off state (e.g., gripper open).  
    - `1`: On state (e.g., gripper closed).  
- **Purpose**:  
  - Coordinates robotic arm motion with external device operations (e.g., triggering gripper actions after reaching target positions).  

## Usage Instructions  

**Set Robot IP Address**  

In `gbt_driver/config/robot_config.yaml`, set `robot_ip_address` to your robot's IP address.  

### Start Service  
```bash  
source install/setup.bash  
ros2 launch gbt_driver gbt_offline_trajectory.launch.py robot_type:=<robot_type>

```  

### Interface Usage Example  
```bash  
ros2 action send_goal gbt_driver/trajectory gbt_interface/action/OfflineTrajectory "{trajectory_path: '/home/gbt/tmp/src/Agilebot_Robot_Ros2/gbt_driver/sample.csv'}"  
```