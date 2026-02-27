# Clone and Build the Project

## 1. Clone the Repository

First, navigate to your ROS2 workspace directory and clone the Agilebot Robotics ROS2 repository: https://github.com/sh-agilebot/Agilebot_Robot_Ros2

```bash
cd {your_ros2_workspace_directory}/src
git clone https://github.com/sh-agilebot/Agilebot_Robot_Ros2.git
```

## 2. Install Python Dependencies

Navigate to the project root directory (the root directory of the cloned repository) and install the required Python dependencies:

```bash
cd {your_ros2_workspace_directory}/src/Agilebot_Robot_Ros2
pip install -r requirements.txt
pip install ./Agilebot.Robot.SDK.A-*.whl
```

> Note: `Agilebot.Robot.SDK.A-*.whl` is the Python SDK package for Agilebot robots, providing libraries required for communication with physical robots. Run the installation command in the repository root (the directory containing the `.whl` file). If multiple SDK versions exist, it is recommended to use the latest one.

# Build the Project

Return to the ROS2 workspace root directory and build the project:

```bash
cd {your_ros2_workspace_directory}
colcon build
source install/setup.bash
```

> **Note**: `colcon build` will automatically build all packages in the workspace, including core packages (gbt_description, gbt_driver, gbt_gazebo, etc.) and MoveIt2 configuration packages (c5a_moveit_config, c7a_moveit_config, c12a_moveit_config, c16a_moveit_config).

### Optional: Selective Build

If you only want to build specific packages, you can use the `--packages-select` parameter:

```bash
# Build only description and driver packages
colcon build --packages-select gbt_description gbt_driver

# Build MoveIt2 configuration for a specific model
colcon build --packages-select c5a_moveit_config
```

You have now successfully installed and configured the Agilebot Robotics ROS2 project!

## Quick Installation Verification

To ensure everything works properly, you can try launching a simple RViz display or Gazebo simulation. For details, please refer to the [Quick Start](../02-quick-start/index.md) page.

---

If you encounter any issues or errors, please check our [FAQ](../04-faq/index.md) page, or submit an issue via [GitHub Issues](https://github.com/sh-agilebot/Agilebot_Robot_Ros2/issues).
