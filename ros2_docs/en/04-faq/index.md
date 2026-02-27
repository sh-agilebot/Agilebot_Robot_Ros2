# FAQ
**Frequently Asked Questions**

## What if I don’t have a physical robot?

If you don’t have access to a real robot, you can use a simulator for testing. Please contact the Shanghai Agilebot team to request access to the cloud-based simulator.

## Is it compatible with other ROS versions?

Currently, only ROS 2 Humble is supported. Other ROS versions are not supported at this time.

## What should I do if I encounter a "package not found" error during build?

Make sure you have installed all dependencies correctly:

```bash
# Install ROS2 dependencies
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-moveit*
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers

# Install Python dependencies
pip install -r requirements.txt
pip install ./Agilebot.Robot.SDK.A-*.whl
```

## How can I check if the installation was successful?

Run the following commands to verify:

```bash
# Check if packages are installed correctly
ros2 pkg list | grep gbt

# Verify URDF file
check_urdf install/gbt_description/share/gbt_description/urdf/GBT_C5A.urdf
```
