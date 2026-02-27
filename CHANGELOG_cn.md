# Changelog

所有主要变更将记录在此文件中。

## [0.2.0] - 2026年2月27日

### Added

- 完善中文文档体系：安装指南、快速开始、示例、FAQ、Roadmap、许可证
- 新增环境与构建说明：Ubuntu 22.04 + ROS2 Humble + Python 3.10 及依赖安装流程
- 明确支持机型：C5A、C7A、C12A、C16A（URDF、Gazebo、MoveIt2）
- 新增真实机器人状态同步说明（发布 `/joint_states` 并在 RViz 显示）
- 新增机器人状态话题说明：`/gbt_driver/feedback_states`
- 新增服务与动作接口文档：`move_to_pose`、IO 读写、程序控制、伺服控制、急停、脚本下发
- 新增 MoveIt2 控制真实机器人（实验性）文档说明
- 新增离线轨迹 CSV 控制说明（含格式定义与示例）
- 新增视觉与应用示例：AgileGaze 通信、视觉码垛 Demo
- 新增连接方式说明：真实机器人、虚拟控制器、AirBot 云机器人

### Changed

- 文档结构统一到 `ros2_docs/zh` 目录并按主题分层
- 快速开始命令与参数说明细化（robot_type、controller_name、enable_rviz）

### Known Limitations

- `/gbt_driver/feedback_states` 的 `io` 字段当前默认不发布，建议通过 `/gbt_driver/service_server/io` 按端口读取
- MoveIt2 控制真实机器人功能仍处于 Experimental 阶段

## [0.1.0] - 2025年8月4日

### Added

- URDF 模型（包含贴图）导入
- RViz 可视化 URDF 模型
- Gazebo 导入 URDF 模型
- MoveIt 轨迹规划
- MoveIt + Gazebo 联合仿真
- 发布机器人信息
  - 发布机器人型号
  - 发布各关节旋转角度
  - 发布末端法兰位姿
  - 发布工具坐标系位姿
  - 发布错误码
  - 发布控制器状态
  - 发布伺服控制器状态
  - 发布机器人连接状态
  - 发布软模式状态
  - 发布全局速度
  - 发布当前激活的 UF、TF
- 将物理机器人姿态同步到 ROS 中
- 通过服务读写 IO
- 通过服务开关 LED
- 通过服务急停
- 通过服务控制程序 start、pause、resume、stop
- 通过服务控制伺服开关
- 通过发送脚本控制机器人
- 通过 action 移动到点
- 离线轨迹
  - 通过 action 执行离线轨迹文件
  - MoveIt2 规划并生成离线轨迹文件执行
- 视觉集成
  - ROS2 节点调用 AgileGaze 流程图
- 码垛
  - 模拟数据的视觉码垛 Demo


