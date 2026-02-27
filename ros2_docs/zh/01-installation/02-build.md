# 克隆与构建项目

## 1. 克隆仓库

首先，导航到您的ROS2工作空间目录，并克隆上海捷勃特机器人的ROS2仓库：https://github.com/sh-agilebot/Agilebot_Robot_Ros2

```bash
cd {你的ros2工作空间目录}/src
git clone https://github.com/sh-agilebot/Agilebot_Robot_Ros2.git
```



## 2. 安装Python依赖

进入项目根目录(上述仓库的根目录)并安装所需的Python依赖：

```bash
cd {你的ros2工作空间目录}/src/Agilebot_Robot_Ros2
pip install -r requirements.txt
pip install ./Agilebot.Robot.SDK.A-*.whl
```

> **说明**: `Agilebot.Robot.SDK.A-*.whl` 是上海捷勃特机器人的 Python SDK 包，包含与物理机器人通信所需的库。请在仓库根目录（包含该 `.whl` 文件的目录）下执行安装命令。若存在多个版本的 SDK 文件，建议使用最新版本。
# 构建项目

返回到ROS2工作空间根目录，然后构建项目：

```bash
cd {你的ros2工作空间目录}
colcon build
source install/setup.bash
```

> **说明**: `colcon build` 会自动构建工作空间中的所有包，包括核心包（gbt_description, gbt_driver, gbt_gazebo 等）和 MoveIt2 配置包（c5a_moveit_config, c7a_moveit_config, c12a_moveit_config, c16a_moveit_config）。

### 可选：选择性构建

如果您只想构建特定的包，可以使用 `--packages-select` 参数：

```bash
# 只构建描述和驱动包
colcon build --packages-select gbt_description gbt_driver

# 只构建特定型号的 MoveIt2 配置
colcon build --packages-select c5a_moveit_config
```

现在，您已经成功安装并配置了上海捷勃特机器人的ROS2项目！

## 快速验证安装

为了确保一切正常，您可以尝试启动一个简单的RViz显示或Gazebo仿真，具体步骤可以参考 [快速开始](../02-quick-start/index.md) 页面。

---

如果有任何问题或遇到错误，请查看我们的 [常见问题](../04-faq/index.md) 页面，或者通过 [GitHub Issues](https://github.com/sh-agilebot/Agilebot_Robot_Ros2/issues) 提交问题。
