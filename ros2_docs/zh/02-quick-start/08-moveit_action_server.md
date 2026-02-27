# MoveIt2控制真实机器人(Experimental)

接收 MoveIt 2 的轨迹消息(follow_joint_trajectory)，并将其转译成捷勃特机器人可执行的离线轨迹文件(csv)，以实现机器人碰撞检测、轨迹规划等功能。
轨迹文件样式请参考：[轨迹文件格式](./10-offline-trajectory.md)

>**注意**：
>
>1.本功能仍处于实验阶段，可能存在抖动或冲击的情况，请谨慎使用。
>
>2.**机器人软模式设置**：本功能需要将机器人设置为 `AUTO` 模式。
>
>   **重要**：真实机器人的软模式（AUTO/MANUAL/MANUAL_LIMIT）**只能通过示教器上的钥匙开关进行物理切换**，没有软件接口可以改变此设置。这是机器人的硬件安全特性。
>
>   - 使用 `get_op_mode()` 可以读取当前模式
>   - **没有** `set_op_mode()` 接口，无法通过软件改变模式
>   - 请在启动前将示教器上的钥匙开关旋转到 `AUTO` 位置
>
>3.请确保机器人软件版本>=7.6.7.0，否则无法使用本功能。
>
>4. `robot_type` 默认值为 `"C5A"`。  可选值：`C7A`、`C12A`、`C16A`。


## 操作指南

**设置机器人IP**

在`gbt_driver/config/robot_config.yaml` 中设置 `robot_ip_address` 字段为机器人的真实IP地址，默认:`10.27.1.254`

### **终端**：
```bash
source install/setup.bash
ros2 launch gbt_driver gbt_action_server.launch.py robot_type:=<robot_type>  interpolate_mode:=quintic
```
> 可选`quintic` 或者 `spline`，默认使用`quintic`插值。
>`quintic` 是一种五次多项式插值方法，而 `spline` 是一种五次B样条插值方法。