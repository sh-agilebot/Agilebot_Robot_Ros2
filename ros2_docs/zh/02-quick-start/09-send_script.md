# 发送脚本到机器人执行

用户可以自行编写符合BasScript语法的脚本，通过本接口发送到机器人执行。建议使用代码生成相应的脚本，防止手动编写错误。

> **已知问题（重点）**
>
> - 本接口只能在 `op_mode=MANUAL`（手动模式）下运行。

> **重要提示：**
>
> - 脚本执行需要机器人处于**正常状态**（无报警）
> - 如果机器人有活动报警，脚本执行将失败并返回错误 `CONTROLLER_INVALID_OPERATION_START`
> - 若控制器状态为 `CTRL_ESTOP`，脚本执行同样会返回 `CONTROLLER_INVALID_OPERATION_START`
> - 请在示教器上清除所有报警后再发送脚本
> - 建议在机器人空闲时调用该服务（避免与 `move_to_pose` 等动作并发）

## 接口说明
| 功能描述 | 发送脚本 |
| :---: | :---- |
| 接口名称 | `gbt_interface/srv/SendScript` |
| 接口说明 | 该接口用于发送脚本到机械臂。 |
| 启动服务 | `ros2 launch gbt_driver gbt_service_server.launch.py robot_type:=<机器人型号> ` |
| 接口使用示例 | `ros2 service call /gbt_driver/service_server/send_script gbt_interface/srv/SendScript "{script_name: 'test', script_content: \"SUB main\\n  MOVEJ PR[1] 100 SD 200.5\\n  RETURN\\nEND\"}"`|

>注意：
>
>1.`script_name: `脚本名称，字符串类型
>
>2.`script_content:` 脚本内容，字符串类型
>
>3.确保`PR[1]`等参数在机器人中存在，否则脚本无法执行
>
>4. `robot_type` 默认值为 `"C5A"`。  可选值：`C7A`、`C12A`、`C16A`。  

## 使用代码生成脚本（推荐）
使用案例如下：
```python
bs = BasScript(name='bas_test')
ret = bs.move_joint(pose_type = MovePoseType.PR, pose_index = 1, speed_type = SpeedType.VALUE, speed_value = 100, smooth_type = SmoothType.SMOOTH_DISTANCE, smooth_distance = 200.5)
bs.content.extend(["  RETURN", "END"])
script="\n".join(bs.content[1:])
```
>更详细的使用方法请参考SDK文档
## 直接编写脚本
```bash
SUB main
  MOVEJ PR[1] 100 SD 200.5
  RETURN
END
```
