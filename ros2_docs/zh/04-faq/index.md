# 常见问题
以下是常见问题，请根据问题进行排查。

## 没有机器人怎么办？
如果您没有机器人，您可以使用模拟器进行测试。请向上海捷勃特机器人的工作人员申请使用云端模拟器。

## 是否兼容其他版本的ROS？
目前仅支持ROS2 Humble版本，其他版本暂不支持。

## 编译时出现 "package not found" 错误怎么办？
确保您已经正确安装了所有依赖：

```bash
# 安装 ROS2 依赖
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-moveit*
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers

# 安装 Python 依赖
pip install -r requirements.txt
pip install ./Agilebot.Robot.SDK.A-*.whl
```

## 如何检查安装是否成功？
运行以下命令验证：

```bash
# 检查包是否正确安装
ros2 pkg list | grep gbt

# 验证 URDF 文件
check_urdf install/gbt_description/share/gbt_description/urdf/GBT_C5A.urdf
```

