# Send the script to the robot for execution
Users can write scripts that conform to the BasScript syntax and send them to the robot for execution via this interface. It is recommended to generate these scripts programmatically to avoid errors from manual editing.

> **Known Issue (Important)**
>
> - This interface can only run in `op_mode=MANUAL` (manual mode).

> **Important Notice:**
>
> - Script execution requires the robot to be in **normal status** (no alarms)
> - If the robot has active alarms, script execution will fail and return error `CONTROLLER_INVALID_OPERATION_START`
> - If controller state is `CTRL_ESTOP`, script execution will also fail with `CONTROLLER_INVALID_OPERATION_START`
> - Please clear all alarms on the teach pendant before sending the script
> - Run this service when the robot is idle (avoid concurrent execution with `move_to_pose`, etc.)

## Interface Description
| **Function Description** | **Send Script** |  
| :---: | :---- |  
| **Interface Name** | `gbt_interface/srv/SendScript` |  
| **Interface Description** |  This interface is used to send scripts to the robotic arm. |  
| **Start Service** | `ros2 launch gbt_driver gbt_service_server.launch.py robot_type:=<robot_type>` |  
| **Usage Example** | `ros2 service call /gbt_driver/service_server/send_script gbt_interface/srv/SendScript "{script_name: 'test', script_content: \"SUB main\\n  MOVEJ PR[1] 100 SD 200.5\\n  RETURN\\nEND\"}"` |  

> **Note:**
> 
> 1. `script_name`: The name of the script (string).
> 2. `script_content`: The content of the script (string).
> 3. Ensure that parameters like `PR[1]` exist on the robot; otherwise, the script will fail to execute.
> 4. `robot_type` defaults to `"C5A"`.  Optional values: `C7A`, `C12A`, `C16A`. 


---

## Use Code to Generate Scripts (Recommended)  
Example usage:  
```python
bs = BasScript(name='bas_test')
ret = bs.move_joint(pose_type=MovePoseType.PR, pose_index=1, speed_type=SpeedType.VALUE, speed_value=100, smooth_type=SmoothType.SMOOTH_DISTANCE, smooth_distance=200.5)
bs.content.extend(["  RETURN", "END"])
script = "\n".join(bs.content[1:])
```
> Detailed instructions for generating scripts are provided in the SDK documentation.


## Direct Script Writing  
```bash
SUB main
  MOVEJ PR[1] 100 SD 200.5
  RETURN
END
```
