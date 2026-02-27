# 上海捷勃特机器人（Agilebot）-ROS2软件包

## 项目概述

本项目提供上海捷勃特机器人的ROS2工具包，旨在帮助用户快速集成和开发机器人应用。工具包包含以下内容：

1. **协作机器人URDF模型**  
   提供标准化URDF描述文件，支持机器人三维建模与仿真。

2. **ROS2驱动包**  
   实现机器人控制与通信功能，支持：
   - 控制程序执行
   - 发送程序脚本给机器人执行
   - 获取机器人状态信息（关节/末端数据、报警信息等）
   - 显示机器人当前姿态

3. **示例代码与教程**  
   包含典型应用案例，指导用户完成，包括但不限于：
   - 使用MoveIt进行运动规划，并转译给捷勃特机器人执行
   - 离线轨迹文件的导入与执行
   - 集成捷勃特的视觉软件AgileGaze
   - 发送自定义程序脚本
   - 视觉码垛demo

该工具包通过提供完整开发资源，降低机器人集成难度，提升开发效率。

## 文档目录
- [安装指南](01-installation/index.md)
- [快速开始](./02-quick-start/index.md)
- [示例](./03-example/index.md)
- [常见问题](./04-faq/index.md)
- [开发计划](./05-roadmap/index.md)
- [许可证](./06-license/index.md)


## 联系方式

如果您有任何建议或希望参与项目的开发，请通过以下方式联系我们：

- **公司官网**：https://www.sh-agilebot.com/
- **邮箱**: info@agilebot.com.cn
- **电话**: `400-996-7588`
- **GitHub**: [https://github.com/sh-agilebot/Agilebot_Robot_Ros2](https://github.com/sh-agilebot/Agilebot_Robot_Ros2)