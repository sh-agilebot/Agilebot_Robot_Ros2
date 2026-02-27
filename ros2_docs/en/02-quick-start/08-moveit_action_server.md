# MoveIt2 Control of a Real Robot (Experimental)

Receive trajectory messages (`follow_joint_trajectory`) from MoveIt 2 and convert them into an offline trajectory file (CSV) executable by the Agilebot robot, enabling robot collision checking and trajectory planning.
Please refer to the [Trajectory File Format](./10-offline-trajectory.md) for the required CSV layout.

> **Note:**
>
> 1. This feature is still in the experimental stage and may cause arm vibration or impact. Please use it with caution.
>
> 2. **Robot Soft Mode Setting**: The robot must be set to **AUTO** mode for this feature to work.
>
>    **Important**: On a real robot, the soft mode (AUTO/MANUAL/MANUAL_LIMIT) **can only be changed using the physical key switch on the teach pendant**. There is **NO software interface** (such as `set_op_mode`) to change this setting - this is a hardware safety feature.
>
>    - Use `get_op_mode()` to read the current mode
>    - **No** `set_op_mode()` interface exists - mode cannot be changed via software
>    - Please turn the key switch on the teach pendant to the **AUTO** position before starting
>
> 3. Please ensure the robot software version is >= 7.6.7.0; otherwise, this feature will not be available.
> 4. `robot_type` defaults to `"C5A"`.  Optional values: `C7A`, `C12A`, `C16A`. 



## Operating Instructions

**Configure the Robot IP**

In `gbt_driver/config/robot_config.yaml`, set the `robot_ip_address` field to the robot’s real IP address. The default is `10.27.1.254`.

### Terminal 

```bash
source install/setup.bash
ros2 launch gbt_driver gbt_action_server.launch.py robot_type:=<robot_type>   interpolate_mode:=quintic
```
> Options: quintic or spline, default is quintic.
> quintic is a quintic polynomial interpolation method, and spline is a quintic B-spline interpolation method.
